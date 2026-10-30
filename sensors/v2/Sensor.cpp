/*
 * Copyright (C) 2019 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Sensor.h"

#include <hardware/sensors.h>
#include <log/log.h>
#include <utils/SystemClock.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>

namespace {

static bool readBool(int fd, bool seek) {
    char c;
    int rc;

    if (seek) {
        rc = lseek(fd, 0, SEEK_SET);
        if (rc) {
            ALOGE("failed to seek: %d", rc);
            return false;
        }
    }

    rc = read(fd, &c, sizeof(c));
    if (rc != 1) {
        ALOGE("failed to read bool: %d", rc);
        return false;
    }

    return c == '1';
}

}  // anonymous namespace

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace subhal {
namespace implementation {

using ::android::hardware::sensors::V1_0::MetaDataEventType;
using ::android::hardware::sensors::V1_0::OperationMode;
using ::android::hardware::sensors::V1_0::Result;
using ::android::hardware::sensors::V1_0::SensorFlagBits;
using ::android::hardware::sensors::V1_0::SensorStatus;
using ::android::hardware::sensors::V2_1::Event;
using ::android::hardware::sensors::V2_1::SensorInfo;
using ::android::hardware::sensors::V2_1::SensorType;

Sensor::Sensor(int32_t sensorHandle, ISensorsEventCallback* callback)
    : mIsEnabled(false),
      mSamplingPeriodNs(0),
      mLastSampleTimeNs(0),
      mStopThread(false),
      mCallback(callback),
      mMode(OperationMode::NORMAL) {
    mSensorInfo.sensorHandle = sensorHandle;
    mSensorInfo.vendor = "The LineageOS Project";
    mSensorInfo.version = 1;
    constexpr float kDefaultMaxDelayUs = 1000 * 1000;
    mSensorInfo.maxDelay = kDefaultMaxDelayUs;
    mSensorInfo.fifoReservedEventCount = 0;
    mSensorInfo.fifoMaxEventCount = 0;
    mSensorInfo.requiredPermission = "";
    mSensorInfo.flags = 0;
}

Sensor::~Sensor() {
    // Ensure that lock is unlocked before calling mRunThread.join() or a
    // deadlock will occur.
    {
        std::unique_lock<std::mutex> lock(mRunMutex);
        mStopThread = true;
        mIsEnabled = false;
        mWaitCV.notify_all();
    }
    if (mRunThread.joinable()) mRunThread.join();
}

const SensorInfo& Sensor::getSensorInfo() const {
    return mSensorInfo;
}

void Sensor::batch(int32_t samplingPeriodNs) {
    samplingPeriodNs =
            std::clamp(samplingPeriodNs, mSensorInfo.minDelay * 1000, mSensorInfo.maxDelay * 1000);

    if (mSamplingPeriodNs != samplingPeriodNs) {
        mSamplingPeriodNs = samplingPeriodNs;
        // Wake up the 'run' thread to check if a new event should be generated now
        mWaitCV.notify_all();
    }
}

Result Sensor::activate(bool enable) {
    std::lock_guard<std::mutex> lock(mRunMutex);
    if (mIsEnabled != enable) {
        mIsEnabled = enable;
        mWaitCV.notify_all();
    }
    return Result::OK;
}

Result Sensor::flush() {
    // Only generate a flush complete event if the sensor is enabled and if the sensor is not a
    // one-shot sensor.
    if (!mIsEnabled) {
        return Result::BAD_VALUE;
    }

    // Note: If a sensor supports batching, write all of the currently batched events for the sensor
    // to the Event FMQ prior to writing the flush complete event.
    Event ev;
    ev.sensorHandle = mSensorInfo.sensorHandle;
    ev.sensorType = SensorType::META_DATA;
    ev.u.meta.what = MetaDataEventType::META_DATA_FLUSH_COMPLETE;
    std::vector<Event> evs{ev};
    mCallback->postEvents(evs, isWakeUpSensor());

    return Result::OK;
}

void Sensor::startThread(Sensor* sensor) {
    sensor->run();
}

void Sensor::run() {
    std::unique_lock<std::mutex> runLock(mRunMutex);
    constexpr int64_t kNanosecondsInSeconds = 1000 * 1000 * 1000;

    while (!mStopThread) {
        if (!mIsEnabled || mMode == OperationMode::DATA_INJECTION) {
            mWaitCV.wait(runLock, [&] {
                return ((mIsEnabled && mMode == OperationMode::NORMAL) || mStopThread);
            });
        } else {
            timespec curTime;
            clock_gettime(CLOCK_REALTIME, &curTime);
            int64_t now = (curTime.tv_sec * kNanosecondsInSeconds) + curTime.tv_nsec;
            int64_t nextSampleTime = mLastSampleTimeNs + mSamplingPeriodNs;

            if (now >= nextSampleTime) {
                mLastSampleTimeNs = now;
                nextSampleTime = mLastSampleTimeNs + mSamplingPeriodNs;
                mCallback->postEvents(readEvents(), isWakeUpSensor());
            }

            mWaitCV.wait_for(runLock, std::chrono::nanoseconds(nextSampleTime - now));
        }
    }
}

bool Sensor::isWakeUpSensor() {
    return mSensorInfo.flags & static_cast<uint32_t>(SensorFlagBits::WAKE_UP);
}

std::vector<Event> Sensor::readEvents() {
    std::vector<Event> events;
    Event event;
    event.sensorHandle = mSensorInfo.sensorHandle;
    event.sensorType = mSensorInfo.type;
    event.timestamp = ::android::elapsedRealtimeNano();
    event.u.vec3.x = 0;
    event.u.vec3.y = 0;
    event.u.vec3.z = 0;
    event.u.vec3.status = SensorStatus::ACCURACY_HIGH;
    events.push_back(event);
    return events;
}

void Sensor::setOperationMode(OperationMode mode) {
    std::lock_guard<std::mutex> lock(mRunMutex);
    if (mMode != mode) {
        mMode = mode;
        mWaitCV.notify_all();
    }
}

bool Sensor::supportsDataInjection() const {
    return mSensorInfo.flags & static_cast<uint32_t>(SensorFlagBits::DATA_INJECTION);
}

Result Sensor::injectEvent(const Event& event) {
    Result result = Result::OK;
    if (event.sensorType == SensorType::ADDITIONAL_INFO) {
        // When in OperationMode::NORMAL, SensorType::ADDITIONAL_INFO is used to push operation
        // environment data into the device.
    } else if (!supportsDataInjection()) {
        result = Result::INVALID_OPERATION;
    } else if (mMode == OperationMode::DATA_INJECTION) {
        mCallback->postEvents(std::vector<Event>{event}, isWakeUpSensor());
    } else {
        result = Result::BAD_VALUE;
    }
    return result;
}

OneShotSensor::OneShotSensor(int32_t sensorHandle, ISensorsEventCallback* callback)
    : Sensor(sensorHandle, callback) {
    mSensorInfo.minDelay = -1;
    mSensorInfo.maxDelay = 0;
    mSensorInfo.flags |= SensorFlagBits::ONE_SHOT_MODE;
}

SysfsPollingOneShotSensor::SysfsPollingOneShotSensor(
        int32_t sensorHandle, ISensorsEventCallback* callback, const std::string& pollPath,
        const std::string& enablePath, const std::string& name, const std::string& typeAsString,
        SensorType type)
    : OneShotSensor(sensorHandle, callback), mEnablePath(enablePath) {
    mSensorInfo.name = name;
    mSensorInfo.type = type;
    mSensorInfo.typeAsString = typeAsString;
    mSensorInfo.maxRange = 2048.0f;
    mSensorInfo.resolution = 1.0f;
    mSensorInfo.power = 0;
    mSensorInfo.flags |= SensorFlagBits::WAKE_UP;

    int rc;

    rc = pipe2(mWaitPipeFd, O_CLOEXEC | O_NONBLOCK);
    if (rc < 0) {
        mWaitPipeFd[0] = -1;
        mWaitPipeFd[1] = -1;
        ALOGE("failed to open wait pipe: %d", rc);
    }

    mPollFd = open(pollPath.c_str(), O_RDONLY | O_CLOEXEC);
    if (mPollFd < 0) {
        ALOGE("failed to open poll fd: %d", mPollFd);
    }

    if (mWaitPipeFd[0] < 0 || mWaitPipeFd[1] < 0 || mPollFd < 0) {
        mStopThread = true;
        return;
    }

    mPolls[0] = {
            .fd = mWaitPipeFd[0],
            .events = POLLIN,
    };

    mPolls[1] = {
            .fd = mPollFd,
            .events = POLLERR | POLLPRI,
    };

    // Consume the initial sysfs notification before arming the sensor.
    readBool(mPollFd, true /* seek */);
    // The base constructor must not start a virtual worker on an incomplete object.
    mRunThread = std::thread(startThread, this);
}

SysfsPollingOneShotSensor::~SysfsPollingOneShotSensor() {
    {
        std::lock_guard<std::mutex> lock(mRunMutex);
        mStopThread = true;
        mIsEnabled = false;
        writeEnable(false);
        mWaitCV.notify_all();
    }
    interruptPoll();
    if (mRunThread.joinable()) mRunThread.join();
    if (mPollFd >= 0) close(mPollFd);
    if (mWaitPipeFd[0] >= 0) close(mWaitPipeFd[0]);
    if (mWaitPipeFd[1] >= 0) close(mWaitPipeFd[1]);
}

bool SysfsPollingOneShotSensor::writeEnable(bool enable) {
    int fd = open(mEnablePath.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        ALOGE("Failed to open %s: %s", mEnablePath.c_str(), strerror(errno));
        return false;
    }
    const char value = enable ? '1' : '0';
    const bool success = TEMP_FAILURE_RETRY(write(fd, &value, sizeof(value))) == 1;
    close(fd);
    if (!success) {
        ALOGE("Failed to write enable to %s", mEnablePath.c_str());
    }
    return success;
}

Result SysfsPollingOneShotSensor::activate(bool enable) {
    std::lock_guard<std::mutex> lock(mRunMutex);
    if (mStopThread) return Result::INVALID_OPERATION;
    if (enable && mIsEnabled) return Result::OK;
    if (enable) {
        // Discard a notification left over from a cancelled registration.
        readFd(mPollFd);
    }
    if (!writeEnable(enable && mMode == OperationMode::NORMAL)) {
        return Result::INVALID_OPERATION;
    }
    mIsEnabled = enable;
    interruptPoll();
    mWaitCV.notify_all();
    return Result::OK;
}

void SysfsPollingOneShotSensor::setOperationMode(OperationMode mode) {
    std::lock_guard<std::mutex> lock(mRunMutex);
    if (mStopThread) return;
    if (!writeEnable(mIsEnabled && mode == OperationMode::NORMAL)) return;
    mMode = mode;
    interruptPoll();
    mWaitCV.notify_all();
}

void SysfsPollingOneShotSensor::run() {
    std::unique_lock<std::mutex> runLock(mRunMutex);

    while (!mStopThread) {
        if (!mIsEnabled || mMode == OperationMode::DATA_INJECTION) {
            mWaitCV.wait(runLock, [&] {
                return ((mIsEnabled && mMode == OperationMode::NORMAL) || mStopThread);
            });
        } else {
            // Cannot hold lock while polling.
            runLock.unlock();
            int rc = TEMP_FAILURE_RETRY(poll(mPolls, 2, -1));
            runLock.lock();

            if (rc < 0) {
                ALOGE("failed to poll: %d", rc);
                mStopThread = true;
                continue;
            }

            if (mPolls[0].revents & POLLIN) {
                char buffer[64];
                while (read(mWaitPipeFd[0], buffer, sizeof(buffer)) > 0) {}
            }
            if (mStopThread || !mIsEnabled || mMode != OperationMode::NORMAL) continue;
            if ((mPolls[1].revents & POLLPRI) && readFd(mPollFd)) {
                // One-shot sensors deactivate before publishing their event.
                mIsEnabled = false;
                writeEnable(false);
                mCallback->postEvents(readEvents(), isWakeUpSensor());
            } else if (mPolls[1].revents & (POLLHUP | POLLNVAL)) {
                ALOGE("Sensor poll fd closed: %s", mSensorInfo.name.c_str());
                mStopThread = true;
            }
        }
    }
}

void SysfsPollingOneShotSensor::interruptPoll() {
    if (mWaitPipeFd[1] < 0) return;

    char c = '1';
    TEMP_FAILURE_RETRY(write(mWaitPipeFd[1], &c, sizeof(c)));
}

std::vector<Event> SysfsPollingOneShotSensor::readEvents() {
    std::vector<Event> events;
    Event event{};
    event.sensorHandle = mSensorInfo.sensorHandle;
    event.sensorType = mSensorInfo.type;
    event.timestamp = ::android::elapsedRealtimeNano();
    fillEventData(event);
    events.push_back(event);
    return events;
}

void SysfsPollingOneShotSensor::fillEventData(Event& event) {
    event.u.data[0] = -1;
    event.u.data[1] = -1;
}

bool SysfsPollingOneShotSensor::readFd(const int fd) {
    return readBool(fd, true /* seek */);
}

void UdfpsSensor::fillEventData(Event& event) {
    event.u.data[0] = mScreenX;
    event.u.data[1] = mScreenY;
}

bool UdfpsSensor::readFd(const int fd) {
    char buffer[512];
    int state = 0;
    int rc;

    rc = lseek(fd, 0, SEEK_SET);
    if (rc < 0) {
        ALOGE("failed to seek: %d", rc);
        return false;
    }
    rc = read(fd, &buffer, sizeof(buffer) - 1);
    if (rc < 0) {
        ALOGE("failed to read state: %d", rc);
        return false;
    }
    buffer[rc] = '\0';
    rc = sscanf(buffer, "%d,%d,%d", &mScreenX, &mScreenY, &state);
    if (rc == 1) {
        // If fod_press_status contains only one value,
        // assume that just reports the state
        state = mScreenX;
        mScreenX = 0;
        mScreenY = 0;
    } else if (rc < 3) {
        ALOGE("failed to parse fp state: %d", rc);
        return false;
    }
    return state > 0;
}

}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
