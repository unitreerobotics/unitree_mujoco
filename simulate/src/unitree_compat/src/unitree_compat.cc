// Clean-room implementation of unitree_sdk2's compiled glue layer, written
// against cyclonedds-cxx built from source. unitree_sdk2 publishes all of its
// DDS wrapper as headers (templates + declarations) but ships the non-template
// definitions only inside a prebuilt linux-only libunitree_sdk2.a; this file
// provides those definitions for platforms (macOS) that binary cannot serve.
// Only symbols reachable from unitree_mujoco's bridge are implemented.

#include <unitree/common/lock/lock.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/time/sleep.hpp>
#include <unitree/common/os.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>
#include <unitree/common/dds/dds_callback.hpp>
#include <unitree/common/dds/dds_qos.hpp>
#include <unitree/common/dds/dds_entity.hpp>
#include <unitree/common/dds/dds_factory_model.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include <condition_variable>
#include <cstring>
#include <mutex>

namespace unitree
{
namespace common
{

/* ------------------------------- lock ---------------------------------- */

Mutex::Mutex()
{
    pthread_mutex_init(&mNative, nullptr);
}

Mutex::~Mutex()
{
    pthread_mutex_destroy(&mNative);
}

void Mutex::Lock()
{
    pthread_mutex_lock(&mNative);
}

void Mutex::Unlock()
{
    pthread_mutex_unlock(&mNative);
}

bool Mutex::Trylock()
{
    return pthread_mutex_trylock(&mNative) == 0;
}

pthread_mutex_t& Mutex::GetNative()
{
    return mNative;
}

Cond::Cond()
{
    pthread_cond_init(&mNative, nullptr);
}

Cond::~Cond()
{
    pthread_cond_destroy(&mNative);
}

void Cond::Wait(Mutex& mutex)
{
    pthread_cond_wait(&mNative, &mutex.GetNative());
}

bool Cond::Wait(Mutex& mutex, uint64_t microsec)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t nsec = static_cast<uint64_t>(ts.tv_nsec) + (microsec % UT_NUMER_MICRO) * 1000;
    ts.tv_sec += static_cast<time_t>(microsec / UT_NUMER_MICRO + nsec / UT_NUMER_NANO);
    ts.tv_nsec = static_cast<long>(nsec % UT_NUMER_NANO);
    return pthread_cond_timedwait(&mNative, &mutex.GetNative(), &ts) == 0;
}

void Cond::Notify()
{
    pthread_cond_signal(&mNative);
}

void Cond::NotifyAll()
{
    pthread_cond_broadcast(&mNative);
}

MutexCond::MutexCond()
{}

MutexCond::~MutexCond()
{}

void MutexCond::Lock()
{
    mMutex.Lock();
}

void MutexCond::Unlock()
{
    mMutex.Unlock();
}

bool MutexCond::Wait(int64_t microsec)
{
    if (microsec <= 0)
    {
        mCond.Wait(mMutex);
        return true;
    }
    return mCond.Wait(mMutex, static_cast<uint64_t>(microsec));
}

void MutexCond::Notify()
{
    mCond.Notify();
}

void MutexCond::NotifyAll()
{
    mCond.NotifyAll();
}

/* ------------------------------- time ---------------------------------- */

void GetCurrentTimeval(struct timeval& tv)
{
    gettimeofday(&tv, nullptr);
}

void GetCurrentTimespec(struct timespec& ts)
{
    clock_gettime(CLOCK_REALTIME, &ts);
}

uint64_t GetCurrentTime()
{
    return static_cast<uint64_t>(time(nullptr));
}

uint64_t GetCurrentTimeNanosecond()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * UT_NUMER_NANO + static_cast<uint64_t>(ts.tv_nsec);
}

uint64_t GetCurrentTimeMicrosecond()
{
    return GetCurrentTimeNanosecond() / 1000;
}

uint64_t GetCurrentTimeMillisecond()
{
    return GetCurrentTimeNanosecond() / UT_NUMER_MICRO;
}

uint64_t GetCurrentCpuTimeNanosecond()
{
    struct timespec ts;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * UT_NUMER_NANO + static_cast<uint64_t>(ts.tv_nsec);
}

uint64_t GetCurrentThreadCpuTimeNanosecond()
{
    struct timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * UT_NUMER_NANO + static_cast<uint64_t>(ts.tv_nsec);
}

uint64_t GetCurrentMonotonicTimeNanosecond()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * UT_NUMER_NANO + static_cast<uint64_t>(ts.tv_nsec);
}

uint64_t GetCurrentCpuTimeMicrosecond()
{
    return GetCurrentCpuTimeNanosecond() / 1000;
}

uint64_t GetCurrentThreadCpuTimeMicrosecond()
{
    return GetCurrentThreadCpuTimeNanosecond() / 1000;
}

uint64_t GetCurrentMonotonicTimeMicrosecond()
{
    return GetCurrentMonotonicTimeNanosecond() / 1000;
}

uint64_t TimevalToMicrosecond(const struct timeval& tv)
{
    return static_cast<uint64_t>(tv.tv_sec) * UT_NUMER_MICRO + static_cast<uint64_t>(tv.tv_usec);
}

uint64_t TimevalToMillisecond(const struct timeval& tv)
{
    return TimevalToMicrosecond(tv) / 1000;
}

uint64_t TimespecToMicrosecond(const struct timespec& ts)
{
    return static_cast<uint64_t>(ts.tv_sec) * UT_NUMER_MICRO + static_cast<uint64_t>(ts.tv_nsec) / 1000;
}

uint64_t TimespecToMillisecond(const struct timespec& ts)
{
    return TimespecToMicrosecond(ts) / 1000;
}

void MicrosecondToTimeval(uint64_t microsec, struct timeval& tv)
{
    tv.tv_sec = static_cast<time_t>(microsec / UT_NUMER_MICRO);
    tv.tv_usec = static_cast<suseconds_t>(microsec % UT_NUMER_MICRO);
}

void MillisecondToTimeval(uint64_t millisec, struct timeval& tv)
{
    MicrosecondToTimeval(millisec * 1000, tv);
}

void MicrosecondToTimespec(uint64_t microsec, struct timespec& ts)
{
    ts.tv_sec = static_cast<time_t>(microsec / UT_NUMER_MICRO);
    ts.tv_nsec = static_cast<long>((microsec % UT_NUMER_MICRO) * 1000);
}

void MillisecondToTimespec(uint64_t millisec, struct timespec& ts)
{
    MicrosecondToTimespec(millisec * 1000, ts);
}

std::string TimeFormatString(struct tm* tmptr, const char* format)
{
    char buf[64];
    snprintf(buf, sizeof(buf), format, tmptr->tm_year + 1900, tmptr->tm_mon + 1,
             tmptr->tm_mday, tmptr->tm_hour, tmptr->tm_min, tmptr->tm_sec);
    return buf;
}

std::string TimeFormatString(struct tm* tmptr, uint64_t precise, const char* format)
{
    char buf[80];
    snprintf(buf, sizeof(buf), format, tmptr->tm_year + 1900, tmptr->tm_mon + 1,
             tmptr->tm_mday, tmptr->tm_hour, tmptr->tm_min, tmptr->tm_sec,
             static_cast<int32_t>(precise));
    return buf;
}

std::string TimeFormatString(uint64_t sec, const char* format)
{
    time_t t = static_cast<time_t>(sec);
    struct tm tmv;
    localtime_r(&t, &tmv);
    return TimeFormatString(&tmv, format);
}

std::string TimeMicrosecondFormatString(const uint64_t& microsec, const char* format)
{
    time_t t = static_cast<time_t>(microsec / UT_NUMER_MICRO);
    struct tm tmv;
    localtime_r(&t, &tmv);
    return TimeFormatString(&tmv, microsec % UT_NUMER_MICRO, format);
}

std::string TimeMillisecondFormatString(const uint64_t& millisec, const char* format)
{
    time_t t = static_cast<time_t>(millisec / UT_NUMER_MILLI);
    struct tm tmv;
    localtime_r(&t, &tmv);
    return TimeFormatString(&tmv, millisec % UT_NUMER_MILLI, format);
}

std::string GetTimeString()
{
    return TimeFormatString(GetCurrentTime());
}

std::string GetTimeMicrosecondString()
{
    return TimeMicrosecondFormatString(GetCurrentTimeMicrosecond(), UT_TIME_MICROSEC_FORMAT_STR);
}

std::string GetTimeMillisecondString()
{
    return TimeMillisecondFormatString(GetCurrentTimeMillisecond(), UT_TIME_MILLISEC_FORMAT_STR);
}

Timer::Timer() : mMicrosecond(0)
{}

Timer::~Timer()
{}

void Timer::Start()
{
    mMicrosecond = GetCurrentMonotonicTimeMicrosecond();
}

void Timer::Restart()
{
    Start();
}

uint64_t Timer::Stop()
{
    return GetCurrentMonotonicTimeMicrosecond() - mMicrosecond;
}

void MicroSleep(uint64_t microsecond)
{
    struct timespec ts;
    MicrosecondToTimespec(microsecond, ts);
    nanosleep(&ts, nullptr);
}

void MilliSleep(uint64_t millisecond)
{
    MicroSleep(millisecond * 1000);
}

void Sleep(uint64_t second)
{
    MicroSleep(second * UT_NUMER_MICRO);
}

/* -------------------------------- os ----------------------------------- */

OsHelper::OsHelper() :
    mUID(getuid()),
    mProcessorNumber(static_cast<int32_t>(sysconf(_SC_NPROCESSORS_ONLN))),
    mProcessorNumberConf(static_cast<int32_t>(sysconf(_SC_NPROCESSORS_CONF))),
    mPageSize(static_cast<int32_t>(sysconf(_SC_PAGESIZE)))
{
    memset(&mPasswd, 0, sizeof(mPasswd));
}

uint32_t OsHelper::GetProcessId()
{
    return static_cast<uint32_t>(getpid());
}

uint64_t OsHelper::GetThreadId()
{
    return static_cast<uint64_t>(pthread_self() ? reinterpret_cast<uintptr_t>(pthread_self()) : 0);
}

int32_t OsHelper::GetTid()
{
#ifdef __APPLE__
    uint64_t tid = 0;
    pthread_threadid_np(nullptr, &tid);
    return static_cast<int32_t>(tid);
#else
    return static_cast<int32_t>(::syscall(SYS_gettid));
#endif
}

/* --------------------------- dds callback ------------------------------ */

DdsReaderCallback::DdsReaderCallback()
{}

DdsReaderCallback::DdsReaderCallback(const DdsMessageHandler& handler) :
    mMessageHandler(handler)
{}

DdsReaderCallback::DdsReaderCallback(const DdsReaderCallback& cb) :
    mMessageHandler(cb.mMessageHandler)
{}

DdsReaderCallback& DdsReaderCallback::operator=(const DdsReaderCallback& cb)
{
    mMessageHandler = cb.mMessageHandler;
    return *this;
}

DdsReaderCallback::~DdsReaderCallback()
{}

bool DdsReaderCallback::HasMessageHandler() const
{
    return static_cast<bool>(mMessageHandler);
}

void DdsReaderCallback::OnDataAvailable(const void* message)
{
    if (mMessageHandler)
    {
        mMessageHandler(message);
    }
}

/* ------------------------------- dds qos -------------------------------- */
// The bridge never sets explicit QoS policies; empty defaults leave the
// cyclonedds native defaults in place, which is also what unitree_sdk2_python
// does (its DataWriter/DataReader are created with qos=None).

#define UT_COMPAT_IMPL_DDS_QOS(QosType)                          \
    void QosType::InitPolicyDefault()                            \
    {}                                                           \
    void QosType::CopyToNativeQos(NativeQosType& qos) const      \
    {                                                            \
        (void)qos;                                               \
    }

UT_COMPAT_IMPL_DDS_QOS(DdsParticipantQos)
UT_COMPAT_IMPL_DDS_QOS(DdsTopicQos)
UT_COMPAT_IMPL_DDS_QOS(DdsPublisherQos)
UT_COMPAT_IMPL_DDS_QOS(DdsSubscriberQos)
UT_COMPAT_IMPL_DDS_QOS(DdsWriterQos)
UT_COMPAT_IMPL_DDS_QOS(DdsReaderQos)

#undef UT_COMPAT_IMPL_DDS_QOS

/* ----------------------------- dds entity ------------------------------- */

DdsLogger::DdsLogger() :
    mLogger(nullptr)
{}

DdsLogger::~DdsLogger()
{}

DdsParticipant::DdsParticipant(uint32_t domainId, const DdsParticipantQos& qos,
                               const std::string& config) :
    mNative(__UT_DDS_NULL__)
{
    UT_DDS_EXCEPTION_TRY
    {
        ::dds::domain::qos::DomainParticipantQos nativeQos;
        qos.CopyToNativeQos(nativeQos);
        if (config.empty())
        {
            mNative = NATIVE_TYPE(domainId, nativeQos);
        }
        else
        {
            mNative = NATIVE_TYPE(domainId, nativeQos, nullptr,
                                  ::dds::core::status::StatusMask::none(), config);
        }
    }
    UT_DDS_EXCEPTION_CATCH(mLogger, true)
}

DdsParticipant::~DdsParticipant()
{
    mNative = __UT_DDS_NULL__;
}

const DdsParticipant::NATIVE_TYPE& DdsParticipant::GetNative() const
{
    return mNative;
}

DdsPublisher::DdsPublisher(const DdsParticipantPtr& participant, const DdsPublisherQos& qos) :
    mNative(__UT_DDS_NULL__)
{
    UT_DDS_EXCEPTION_TRY
    {
        ::dds::pub::qos::PublisherQos nativeQos = participant->GetNative().default_publisher_qos();
        qos.CopyToNativeQos(nativeQos);
        mNative = NATIVE_TYPE(participant->GetNative(), nativeQos);
    }
    UT_DDS_EXCEPTION_CATCH(mLogger, true)
}

DdsPublisher::~DdsPublisher()
{
    mNative = __UT_DDS_NULL__;
}

const DdsPublisher::NATIVE_TYPE& DdsPublisher::GetNative() const
{
    return mNative;
}

DdsSubscriber::DdsSubscriber(const DdsParticipantPtr& participant, const DdsSubscriberQos& qos) :
    mNative(__UT_DDS_NULL__)
{
    UT_DDS_EXCEPTION_TRY
    {
        ::dds::sub::qos::SubscriberQos nativeQos = participant->GetNative().default_subscriber_qos();
        qos.CopyToNativeQos(nativeQos);
        mNative = NATIVE_TYPE(participant->GetNative(), nativeQos);
    }
    UT_DDS_EXCEPTION_CATCH(mLogger, true)
}

DdsSubscriber::~DdsSubscriber()
{
    mNative = __UT_DDS_NULL__;
}

const DdsSubscriber::NATIVE_TYPE& DdsSubscriber::GetNative() const
{
    return mNative;
}

/* --------------------------- dds factory model --------------------------- */

DdsFactoryModel::DdsFactoryModel() :
    mLogger(nullptr)
{}

DdsFactoryModel::~DdsFactoryModel()
{}

void DdsFactoryModel::Init(uint32_t domainId, const std::string& ddsConfig)
{
    mParticipant = std::make_shared<DdsParticipant>(domainId, mParticipantQos, ddsConfig);
    mPublisher = std::make_shared<DdsPublisher>(mParticipant, mPublisherQos);
    mSubscriber = std::make_shared<DdsSubscriber>(mParticipant, mSubscriberQos);
}

/* ------------------------------- thread --------------------------------- */

namespace
{
class CompatFuture : public Future
{
public:
    int32_t GetState() override
    {
        std::lock_guard<std::mutex> lock(mMutex);
        return mState;
    }

    bool Wait(int64_t microsec) override
    {
        std::unique_lock<std::mutex> lock(mMutex);
        if (microsec <= 0)
        {
            mCond.wait(lock, [this] { return mState != DEFER; });
        }
        else
        {
            mCond.wait_for(lock, std::chrono::microseconds(microsec),
                           [this] { return mState != DEFER; });
        }
        return mState != DEFER;
    }

    const Any& GetValue(int64_t microsec) override
    {
        Wait(microsec);
        return mValue;
    }

    const Any& GetFaultMessage() override
    {
        return mFault;
    }

    void Ready(const Any& value) override
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mValue = value;
        mState = READY;
        mCond.notify_all();
    }

    void Fault(const Any& message) override
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mFault = message;
        mState = FAULT;
        mCond.notify_all();
    }

private:
    std::mutex mMutex;
    std::condition_variable mCond;
    int32_t mState = DEFER;
    Any mValue;
    Any mFault;
};

void* CompatThreadEntry(void* arg)
{
    static_cast<Thread*>(arg)->Wrap();
    return nullptr;
}
} // namespace

FutureWrapper::FutureWrapper()
{
    mFuturePtr = std::make_shared<CompatFuture>();
}

FutureWrapper::~FutureWrapper()
{}

Thread::~Thread()
{
    if (mThreadId != 0)
    {
        pthread_join(mThreadId, nullptr);
        mThreadId = 0;
    }
}

uint64_t Thread::GetThreadId() const
{
    return reinterpret_cast<uint64_t>(mThreadId);
}

void Thread::SetCpu()
{
    // No thread-to-core pinning on macOS; the bridge passes UT_CPU_ID_NONE anyway.
}

void Thread::SetName()
{
#ifdef __APPLE__
    if (!mName.empty())
    {
        pthread_setname_np(mName.c_str());
    }
#else
    if (!mName.empty())
    {
        pthread_setname_np(pthread_self(), mName.c_str());
    }
#endif
}

void Thread::SetPriority(int32_t priority)
{
    (void)priority;
}

void Thread::Wrap()
{
    SetName();
    SetCpu();
    try
    {
        Any result = mFunc ? mFunc() : Any();
        Ready(result);
    }
    catch (const std::exception& e)
    {
        Fault(Any(std::string(e.what())));
    }
    catch (...)
    {
        Fault(Any(std::string("unknown exception")));
    }
}

void Thread::CreateThreadNative()
{
    if (pthread_create(&mThreadId, nullptr, &CompatThreadEntry, this) != 0)
    {
        mThreadId = 0;
        UT_THROW(SystemException, "pthread_create failed");
    }
}

RecurrentThread::~RecurrentThread()
{
    mQuit = true;
    // Thread::~Thread joins the pthread after this returns.
}

int32_t RecurrentThread::ThreadFunc()
{
    while (!mQuit)
    {
        uint64_t start = GetCurrentMonotonicTimeMicrosecond();
        if (mFunc)
        {
            mFunc();
        }
        uint64_t elapsed = GetCurrentMonotonicTimeMicrosecond() - start;
        if (elapsed < mIntervalMicrosec)
        {
            MicroSleep(mIntervalMicrosec - elapsed);
        }
    }
    return 0;
}

int32_t RecurrentThread::ThreadFunc_0()
{
    while (!mQuit)
    {
        if (mFunc)
        {
            mFunc();
        }
    }
    return 0;
}

bool RecurrentThread::Wait(int64_t microsec)
{
    return FutureWrapper::Wait(microsec);
}

} // namespace common

/* ---------------------------- channel factory ---------------------------- */

namespace robot
{

namespace
{
const char kAutoDetermineConfig[] =
    "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
    "<CycloneDDS>"
    "  <Domain Id=\"any\">"
    "    <General>"
    "      <Interfaces>"
    "        <NetworkInterface autodetermine=\"true\" priority=\"default\" multicast=\"default\"/>"
    "      </Interfaces>"
    "    </General>"
    "  </Domain>"
    "</CycloneDDS>";

const char kInterfaceConfigPrefix[] =
    "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
    "<CycloneDDS>"
    "  <Domain Id=\"any\">"
    "    <General>"
    "      <Interfaces>"
    "        <NetworkInterface name=\"";

const char kInterfaceConfigSuffix[] =
    "\" priority=\"default\" multicast=\"default\"/>"
    "      </Interfaces>"
    "    </General>"
    "  </Domain>"
    "</CycloneDDS>";
} // namespace

ChannelFactory::ChannelFactory() :
    mInited(false)
{}

ChannelFactory::~ChannelFactory()
{}

void ChannelFactory::Init(int32_t domainId, const std::string& networkInterface)
{
    common::LockGuard<common::Mutex> guard(mMutex);
    if (mInited)
    {
        return;
    }

    std::string selectedInterface = networkInterface;
#ifdef __APPLE__
    // The repository's cross-platform default is Linux's loopback name.
    if (selectedInterface == "lo")
    {
        selectedInterface = "lo0";
    }
#endif

    std::string config;
    if (selectedInterface.empty())
    {
        config = kAutoDetermineConfig;
    }
    else
    {
        config = std::string(kInterfaceConfigPrefix) + selectedInterface + kInterfaceConfigSuffix;
    }

    mDdsFactoryPtr = std::make_shared<common::DdsFactoryModel>();
    mDdsFactoryPtr->Init(static_cast<uint32_t>(domainId), config);
    mInited = true;
}

void ChannelFactory::Release()
{
    common::LockGuard<common::Mutex> guard(mMutex);
    mDdsFactoryPtr.reset();
    mInited = false;
}

} // namespace robot
} // namespace unitree
