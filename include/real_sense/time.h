/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_IO_DEPTH_SENSE_TIME_H
#define PCL_IO_DEPTH_SENSE_TIME_H

#include <queue>

#include <pcl/pcl_config.h>
#include <pcl/common/time.h>

namespace pcl
{

#if PCL_VERSION < 100800

  /** \brief A helper class to measure frequency of a certain event.
    *
    * To use this class create an instance and call event() function every time
    * the event in question occurs. The estimated frequency can be retrieved
    * with getFrequency() function.
    *
    * \author Sergey Alexandrov
    * \ingroup common
    */
  class EventFrequency
  {

    public:

      /** \brief Constructor.
        *
        * \param[i] window_size number of most recent events that are
        * considered in frequency estimation (default: 30) */
      EventFrequency (size_t window_size = 30)
      : window_size_ (window_size)
      {
        stop_watch_.reset ();
      }

      /** \brief Notifies the class that the event occured. */
      void event ()
      {
        event_time_queue_.push (stop_watch_.getTimeSeconds ());
        if (event_time_queue_.size () > window_size_)
          event_time_queue_.pop ();
      }

      /** \brief Retrieve the estimated frequency. */
      double
      getFrequency () const
      {
        if (event_time_queue_.size () < 2)
          return (0.0);
        return ((event_time_queue_.size () - 1) /
                (event_time_queue_.back () - event_time_queue_.front ()));
      }

      /** \brief Reset frequency computation. */
      void reset ()
      {
        stop_watch_.reset ();
        event_time_queue_ = std::queue<double> ();
      }

    private:

      pcl::StopWatch stop_watch_;
      std::queue<double> event_time_queue_;
      const size_t window_size_;

  };

#endif

  class MyStopWatch
  {
    public:
      /** \brief Constructor. */
      MyStopWatch () : start_time_ (boost::posix_time::microsec_clock::local_time ())
      {
      }

      /** \brief Destructor. */
      virtual ~MyStopWatch () {}

      /** \brief Retrieve the time in milliseconds spent since the last call to \a reset(). */
      inline double
      getTime ()
      {
        boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time ();
        return (static_cast<double> (((end_time - start_time_).total_milliseconds ())));
      }

      /** \brief Retrieve the time in seconds spent since the last call to \a reset(). */
      inline double
      getTimeSeconds ()
      {
        return (getTime () * 0.001f);
      }

      inline double
      getTimeMicroSeconds ()
      {
        boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time ();
        return (static_cast<double> (((end_time - start_time_).total_microseconds ())));
      }

      /** \brief Reset the stopwatch to 0. */
      inline void
      reset ()
      {
        start_time_ = boost::posix_time::microsec_clock::local_time ();
      }

    protected:
      boost::posix_time::ptime start_time_;
  };

  class MyScopeTime : public MyStopWatch
  {
    public:
      inline MyScopeTime (const char* title) : 
        title_ (std::string (title))
      {
        start_time_ = boost::posix_time::microsec_clock::local_time ();
      }

      inline MyScopeTime () :
        title_ (std::string (""))
      {
        start_time_ = boost::posix_time::microsec_clock::local_time ();
      }

      inline ~MyScopeTime ()
      {
        double val = this->getTimeMicroSeconds ();
        std::cerr << title_ << " took " << val << " microseconds.\n";
      }

    private:
      std::string title_;
  };

}

#endif /* PCL_IO_DEPTH_SENSE_TIME_H */

