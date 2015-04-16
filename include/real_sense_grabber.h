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

#ifndef PCL_IO_REAL_SENSE_GRABBER_H
#define PCL_IO_REAL_SENSE_GRABBER_H

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/grabber.h>

#include "real_sense/time.h"

namespace pcl
{

  namespace io
  {

    template <typename T> class Buffer;

    namespace real_sense
    {
      class RealSenseDevice;
    }

  }

  class PCL_EXPORTS RealSenseGrabber : public Grabber
  {

    public:

      typedef
        void (sig_cb_real_sense_point_cloud)
          (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);

      typedef
        void (sig_cb_real_sense_point_cloud_rgba)
          (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&);

      enum Mode
      {
        RealSense_VGA_30Hz = 0,
      };

      enum TemporalFilteringType
      {
        RealSense_None = 0,
        RealSense_Median = 1,
        RealSense_Average = 2,
      };

      /** Create a grabber for a RealSense device.
        *
        * The grabber "captures" the device, making it impossible for other
        * grabbers to interact with it. The device is "released" when the
        * grabber is destructed.
        *
        * This will throw pcl::io::IOException if there are no free devices
        * that match the supplied \a device_id.
        *
        * \param[in] device_id device identifier, which can be a serial number,
        * an index (with '#' prefix), or an empty string (to select the first
        * available device) */
      RealSenseGrabber (const std::string& device_id = "");

      virtual
      ~RealSenseGrabber () throw ();

      virtual void
      start ();

      virtual void
      stop ();

      virtual bool
      isRunning () const;

      virtual std::string
      getName () const
      {
        return (std::string ("RealSenseGrabber"));
      }

      virtual float
      getFramesPerSecond () const;

      void
      setConfidenceThreshold (unsigned int threshold);

      void
      enableTemporalFiltering (TemporalFilteringType type, size_t window_size);

      void
      disableTemporalFiltering ();

      const std::string&
      getDeviceSerialNumber () const;

    private:

      void run ();

      // Signals to indicate whether new clouds are available
      boost::signals2::signal<sig_cb_real_sense_point_cloud>* point_cloud_signal_;
      boost::signals2::signal<sig_cb_real_sense_point_cloud_rgba>* point_cloud_rgba_signal_;

      boost::shared_ptr<pcl::io::real_sense::RealSenseDevice> device_;

      bool is_running_;
      unsigned int confidence_threshold_;
      TemporalFilteringType temporal_filtering_type_;

      /// Indicates whether there are subscribers for PointXYZ signal, computed
      /// and stored on start()
      bool need_xyz_;

      /// Indicates whether there are subscribers for PointXYZRGBA signal,
      /// computed and stored on start()
      bool need_xyzrgba_;

      EventFrequency frequency_;
      mutable boost::mutex fps_mutex_;

      boost::thread thread_;

      static const int FRAMERATE = 30;
      static const int WIDTH = 640;
      static const int HEIGHT = 480;
      static const int SIZE = WIDTH * HEIGHT;
      static const int COLOR_WIDTH = 640;
      static const int COLOR_HEIGHT = 480;
      static const int COLOR_SIZE = COLOR_WIDTH * COLOR_HEIGHT;

      /// Depth buffer to perform temporal filtering of the depth images
      boost::shared_ptr<pcl::io::Buffer<unsigned short> > depth_buffer_;

  };

}

#endif /* PCL_IO_REAL_SENSE_GRABBER_H */

