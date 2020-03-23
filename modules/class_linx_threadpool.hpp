#ifndef CLASS_LINX_THREAD_POOL_HPP_
#define CLASS_LINX_THREAD_POOL_HPP_
#include <functional>
#include <iostream>
#include <thread>
#include <vector>
#include "boost/threadpool.hpp"

class LinxThreadPool
{
public:
	LinxThreadPool()
	{

	}
	~LinxThreadPool()
	{
		if (tp)
		{
			delete tp;
		}
	}

	void set_thread_size(int n)
	{
		tp = new boost::threadpool::pool(n);
	}

	template<class F>
	void enqueue(F f)
	{
		tp->schedule(f);
	}

	void wait_all()
	{
		tp->wait();
	}
private:

	boost::threadpool::pool *tp;
};
#endif
