/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

/** @file packet_pipeline.cpp Packet pipeline implementation. */

#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/async_packet_processor.h>
#include <libfreenect2/data_callback.h>
#include <libfreenect2/rgb_packet_stream_parser.h>
#include <libfreenect2/depth_packet_stream_parser.h>
#include <libfreenect2/protocol/response.h>

namespace libfreenect2
{

static RgbPacketProcessor *getDefaultRgbPacketProcessor()
{
#if defined(LIBFREENECT2_WITH_VT_SUPPORT)
  return new VTRgbPacketProcessor();
#elif defined(LIBFREENECT2_WITH_VAAPI_SUPPORT)
  RgbPacketProcessor *vaapi = new VaapiRgbPacketProcessor();
  if (vaapi->good())
    return vaapi;
  else
    delete vaapi;
  return new TurboJpegRgbPacketProcessor();
#elif defined(LIBFREENECT2_WITH_TEGRAJPEG_SUPPORT)
  RgbPacketProcessor *tegra = new TegraJpegRgbPacketProcessor();
  if (tegra->good())
    return tegra;
  else
    delete tegra;
  return new TurboJpegRgbPacketProcessor();
#elif defined(LIBFREENECT2_WITH_MEDIASDK_SUPPORT)
  RgbPacketProcessor *mediasdk = new MediaSdkRgbPacketProcessor();
  if (mediasdk->good())
    return mediasdk;
  else
    delete mediasdk;
  return new TurboJpegRgbPacketProcessor();
#elif defined(LIBFREENECT2_WITH_TURBOJPEG_SUPPORT)
  return new TurboJpegRgbPacketProcessor();
#else
  #error No jpeg decoder is enabled
#endif
}

// this function allows the caller to specify what type of RGB processor they
// want
static RgbPacketProcessor *getSpecificRgbPacketProcessor(const std::string &rgbProcessor)
{
  printf("Trying to get specific RGB processor: %s\n", rgbProcessor);
  std::string lowercaseProcessor;
  std::transform(rgbProcessor.begin(), rgbProcessor.end(), lowercaseProcessor.begin(), ::tolower);
  if(rgbProcessor.compare("vt") == 0)
  {
#if defined(LIBFREENECT2_WITH_VT_SUPPORT)
    return new VTRgbPacketProcessor();
#else
	// they have requested VT but this isn't supported so we have to fall back to default
	return getDefaultRgbPacketProcessor();
#endif
  }
  else if(rgbProcessor.compare("vaapi") == 0)
  {
#if defined(LIBFREENECT2_WITH_VAAPI_SUPPORT)
	RgbPacketProcessor *vaapi = new VaapiRgbPacketProcessor();
	if (vaapi->good())
		return vaapi;
	else
		delete vaapi;
	return new TurboJpegRgbPacketProcessor();
#else
	// they have requested VAAPI but this isn't supported so we have to fall back to default
	return getDefaultRgbPacketProcessor();
#endif
  }
  else if(rgbProcessor.compare("tegra") == 0)
  {
#if defined(LIBFREENECT2_WITH_TEGRAJPEG_SUPPORT)
	  RgbPacketProcessor *tegra = new TegraJpegRgbPacketProcessor();
	  if (tegra->good())
		return tegra;
	  else
		delete tegra;
	  return new TurboJpegRgbPacketProcessor();
#else
	// they have requested TEGRA but this isn't supported so we have to fall back to default
	return getDefaultRgbPacketProcessor();
#endif
  }
  else if(rgbProcessor.compare("mediasdk") == 0)
  {
#if defined(LIBFREENECT2_WITH_MEDIASDK_SUPPORT)
  RgbPacketProcessor *mediasdk = new MediaSdkRgbPacketProcessor();
  if (mediasdk->good())
    return mediasdk;
  else
    delete mediasdk;
  return new TurboJpegRgbPacketProcessor();
#else
	// they have requested MEDIASDK but this isn't supported so we have to fall back to default
	return getDefaultRgbPacketProcessor();
#endif
  }
  else if(rgbProcessor.compare("turbo") == 0)
  {
#if defined(LIBFREENECT2_WITH_TURBOJPEG_SUPPORT)
  return new TurboJpegRgbPacketProcessor();
#else
	// they have requested MEDIASDK but this isn't supported so we have to fall back to default
	return getDefaultRgbPacketProcessor();
#endif
  }
  else
  {
	return getDefaultRgbPacketProcessor();
  }
}

class PacketPipelineComponents
{
public:
  RgbPacketStreamParser *rgb_parser_;
  DepthPacketStreamParser *depth_parser_;

  RgbPacketProcessor *rgb_processor_;
  BaseRgbPacketProcessor *async_rgb_processor_;
  DepthPacketProcessor *depth_processor_;
  BaseDepthPacketProcessor *async_depth_processor_;

  ~PacketPipelineComponents();
  void initialize(RgbPacketProcessor *rgb, DepthPacketProcessor *depth);
};

void PacketPipelineComponents::initialize(RgbPacketProcessor *rgb, DepthPacketProcessor *depth)
{
  rgb_parser_ = new RgbPacketStreamParser();
  depth_parser_ = new DepthPacketStreamParser();

  rgb_processor_ = rgb;
  depth_processor_ = depth;

  async_rgb_processor_ = new AsyncPacketProcessor<RgbPacket>(rgb_processor_);
  async_depth_processor_ = new AsyncPacketProcessor<DepthPacket>(depth_processor_);

  rgb_parser_->setPacketProcessor(async_rgb_processor_);
  depth_parser_->setPacketProcessor(async_depth_processor_);
}

PacketPipelineComponents::~PacketPipelineComponents()
{
  delete async_rgb_processor_;
  delete async_depth_processor_;
  delete rgb_processor_;
  delete depth_processor_;
  delete rgb_parser_;
  delete depth_parser_;
}

PacketPipeline::PacketPipeline(): comp_(new PacketPipelineComponents()) {}

PacketPipeline::~PacketPipeline()
{
  delete comp_;
}

PacketPipeline::PacketParser *PacketPipeline::getRgbPacketParser() const
{
  return comp_->rgb_parser_;
}

PacketPipeline::PacketParser *PacketPipeline::getIrPacketParser() const
{
  return comp_->depth_parser_;
}

RgbPacketProcessor *PacketPipeline::getRgbPacketProcessor() const
{
  return comp_->rgb_processor_;
}

DepthPacketProcessor *PacketPipeline::getDepthPacketProcessor() const
{
  return comp_->depth_processor_;
}

CpuPacketPipeline::CpuPacketPipeline(const std::string &rgbProcessorType)
{
  if(rgbProcessorType.empty())
    comp_->initialize(getDefaultRgbPacketProcessor(), new CpuDepthPacketProcessor());
  else
	comp_->initialize(getSpecificRgbPacketProcessor(rgbProcessorType), new CpuDepthPacketProcessor());
}

CpuPacketPipeline::~CpuPacketPipeline() { }

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
OpenGLPacketPipeline::OpenGLPacketPipeline(const std::string &rgbProcessorType, void *parent_opengl_context, bool debug) : parent_opengl_context_(parent_opengl_context), debug_(debug)
{
  if(rgbProcessorType.empty())
    comp_->initialize(getDefaultRgbPacketProcessor(), new OpenGLDepthPacketProcessor(parent_opengl_context_, debug_));
  else
	comp_->initialize(getSpecificRgbPacketProcessor(rgbProcessorType), new OpenGLDepthPacketProcessor(parent_opengl_context_, debug_));
}

OpenGLPacketPipeline::~OpenGLPacketPipeline() { }
#endif // LIBFREENECT2_WITH_OPENGL_SUPPORT


#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
OpenCLPacketPipeline::OpenCLPacketPipeline(const std::string &rgbProcessorType, const int deviceId) : deviceId(deviceId)
{
  if(rgbProcessorType.empty())
    comp_->initialize(getDefaultRgbPacketProcessor(), new OpenCLDepthPacketProcessor(deviceId));
  else
    comp_->initialize(getSpecificRgbPacketProcessor(rgbProcessorType), new OpenCLDepthPacketProcessor(deviceId));
}

OpenCLPacketPipeline::~OpenCLPacketPipeline() { }


OpenCLKdePacketPipeline::OpenCLKdePacketPipeline(const int deviceId) : deviceId(deviceId)
{
  comp_->initialize(getDefaultRgbPacketProcessor(), new OpenCLKdeDepthPacketProcessor(deviceId));
}

OpenCLKdePacketPipeline::~OpenCLKdePacketPipeline() { }
#endif // LIBFREENECT2_WITH_OPENCL_SUPPORT

#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
CudaPacketPipeline::CudaPacketPipeline(const std::string &rgbProcessorType, const int deviceId) : deviceId(deviceId)
{
  if(rgbProcessorType.empty())
    comp_->initialize(getDefaultRgbPacketProcessor(), new CudaDepthPacketProcessor(deviceId));
  else
    comp_->initialize(getSpecificRgbPacketProcessor(rgbProcessorType), new CudaDepthPacketProcessor(deviceId));
}

CudaPacketPipeline::~CudaPacketPipeline() { }

CudaKdePacketPipeline::~CudaKdePacketPipeline() { }

CudaKdePacketPipeline::CudaKdePacketPipeline(const int deviceId) : deviceId(deviceId)
{
  comp_->initialize(getDefaultRgbPacketProcessor(), new CudaKdeDepthPacketProcessor(deviceId));
}
#endif // LIBFREENECT2_WITH_CUDA_SUPPORT

DumpPacketPipeline::DumpPacketPipeline()
{
  RgbPacketProcessor *rgb = new DumpRgbPacketProcessor();
  DepthPacketProcessor *depth = new DumpDepthPacketProcessor();
  comp_->initialize(rgb, depth);
}

DumpPacketPipeline::~DumpPacketPipeline() {}

const unsigned char* DumpPacketPipeline::getDepthP0Tables(size_t* length) {
  *length = sizeof(libfreenect2::protocol::P0TablesResponse);
  return static_cast<DumpDepthPacketProcessor*>(getDepthPacketProcessor())->getP0Tables();
}

const float* DumpPacketPipeline::getDepthXTable(size_t* length) {
  *length = DepthPacketProcessor::TABLE_SIZE;
  return static_cast<DumpDepthPacketProcessor*>(getDepthPacketProcessor())->getXTable();
}

const float* DumpPacketPipeline::getDepthZTable(size_t* length) {
  *length = DepthPacketProcessor::TABLE_SIZE;
  return static_cast<DumpDepthPacketProcessor*>(getDepthPacketProcessor())->getZTable();
}

const short* DumpPacketPipeline::getDepthLookupTable(size_t* length) {
  *length = DepthPacketProcessor::LUT_SIZE;
  return static_cast<DumpDepthPacketProcessor*>(getDepthPacketProcessor())->getLookupTable();
}

} /* namespace libfreenect2 */
