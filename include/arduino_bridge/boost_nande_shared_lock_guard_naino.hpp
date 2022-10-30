#pragma once

#include <boost/thread.hpp>

namespace arduino_bridge
{
	class SharedLockGuard final
	{
		boost::shared_mutex& m;
		SharedLock(boost::shared_mutex& m):
			m{m}
		{
			m.lock_shared();
		}

		~SharedLockGuard()
		{
			m.unlock_
		}
	};
}