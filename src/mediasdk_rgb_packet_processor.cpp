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

// TO LOOK INTO - JASON TO REMOVE:
// - Look at the "low_latency" option in the sample decoder (only works for H.264 and JPEG)
// - Look at the "threads_num" option in the sample decoder
// - The height of the image is 1088 instead of 1080 so some needs to be cropped off. Its easy to
//   crop in terms of height as we can just skip entire rows and still use a simple memcpy. Need to
//   double check that our current cropped image (which will be missing some rows) is giving the same
//   as the old turbo JPEG decoder

// USEFUL RESOURCES FOR DEVELOPING USING THE INTEL MEDIA SDK
// https://software.intel.com/en-us/articles/framework-for-developing-applications-using-media-sdk

#ifdef LIBFREENECT2_WITH_MEDIASDK_SUPPORT

#include <libfreenect2/rgb_packet_processor.h>

#include <cstring>
#include <cstdio>

#include <chrono>
#include <thread>

// Intel Media SDK header
#include "mfxcommon.h"
#include "mfxmvc.h"
#include "mfxjpeg.h"
#include "mfxplugin.h"
#include "mfxplugin++.h"
#include "mfxvideo.h"
#include "mfxvideo++.h"
// also have to link in libmfx.lib

#include "common_utils.h"
#include "common_directx11.h"

#include "libfreenect2/logging.h"
#include "libfreenect2/allocator.h"

#define CHECK_COND(cond) do { if (!(cond)) { LOG_ERROR << #cond " failed"; return false; } } while(0)
#define CHECK_VA(expr) do { VAStatus err = (expr); if (err != VA_STATUS_SUCCESS) { LOG_ERROR << #expr ": " << vaErrorStr(err); return false; } } while(0)
#define CALL_VA(expr) do { VAStatus err = (expr); if (err != VA_STATUS_SUCCESS) { LOG_ERROR << #expr ": " << vaErrorStr(err); } } while(0)

namespace libfreenect2
{
	// a bunch of macros used by the Intel Media SDK 
#define MSDK_PRINT_RET_MSG(ERR)         {PrintErrString(ERR, __FILE__, __LINE__);}
#define MSDK_CHECK_RESULT(P, X, ERR)    {if ((X) > (P)) {MSDK_PRINT_RET_MSG(ERR); return ERR;}}
#define MSDK_CHECK_POINTER(P, ERR)      {if (!(P)) {MSDK_PRINT_RET_MSG(ERR); return ERR;}}
#define MSDK_CHECK_ERROR(P, X, ERR)     {if ((X) == (P)) {MSDK_PRINT_RET_MSG(ERR); return ERR;}}
#define MSDK_IGNORE_MFX_STS(P, X)       {if ((X) == (P)) {P = MFX_ERR_NONE;}}
#define MSDK_BREAK_ON_ERROR(P)          {if (MFX_ERR_NONE != (P)) break;}
#define MSDK_SAFE_DELETE_ARRAY(P)       {if (P) {delete[] P; P = NULL;}}
#define MSDK_ALIGN32(X)                 (((mfxU32)((X)+31)) & (~ (mfxU32)31))
#define MSDK_ALIGN16(value)             (((value + 15) >> 4) << 4)
#define MSDK_SAFE_RELEASE(X)            {if (X) { X->Release(); X = NULL; }}
#define MSDK_MAX(A, B)                  (((A) > (B)) ? (A) : (B))


// Usage of the following two macros are only required for certain Windows DirectX11 use cases
//#define WILL_READ  0x1000 // this is from the Intel Media SDK tutorial file common_utils.h

	class MediaSdkRgbPacketProcessorImpl: public WithPerfLogging
	{
	public:
		Frame *frame;

		MFXVideoSession m_mfxSession;
		mfxIMPL m_implementation;
		mfxBitstream m_mfxBS; // contains encoded data
		mfxVideoParam m_mfxVideoParams;
		MFXVideoDECODE* m_pmfxDEC;

		mfxIMPL actualImplementation;
		mfxVersion actualVersion;

		// the surfaces we'll use for decoding
		mfxFrameAllocator mfxAllocator;
		mfxU32 surfaceSize;
		mfxU16 numSurfaces;
		mfxU8* surfaceBuffers;
		mfxFrameSurface1** pmfxSurfaces;

		bool useVideoMemory;

		mfxU32 m_fourcc; // color format of vpp out, i420 by default
		//MemType m_memType; // memory type of surfaces to use. Not sure if this is needed
		mfxU32 m_nMaxFps; // limit of fps, if isn't specified equal 0.
		mfxU32 m_nFrames; //limit number of output frames.. set to infinite for our use case

		static const int WIDTH = 1920;
		static const int HEIGHT = 1080;

		bool decodedFirstHeader;

		bool good;

		MediaSdkRgbPacketProcessorImpl()
		{
			printf("Using Media SDK\n");

			useVideoMemory = false;

			good = false;

			decodedFirstHeader = false;

			m_pmfxDEC = NULL;

			memset(&m_mfxVideoParams, 0, sizeof(m_mfxVideoParams));
			memset(&m_mfxBS, 0, sizeof(m_mfxBS));

			m_implementation = 0;

			numSurfaces = 0;
			surfaceBuffers = 0;
			pmfxSurfaces = 0;

			good = initializeMediaSdk();
			if (!good)
				return;

			newFrame();
		}

		~MediaSdkRgbPacketProcessorImpl()
		{
			if(frame != NULL)
				delete frame;

			if(m_pmfxDEC != NULL)
			{
				m_pmfxDEC->Close();
				delete m_pmfxDEC;
			}

			for (int i = 0; i < numSurfaces; i++)
				delete pmfxSurfaces[i];

			if(pmfxSurfaces)
				delete [] pmfxSurfaces;

			if(surfaceBuffers)
				delete [] surfaceBuffers;
		}

		

		void newFrame()
		{
			// not sure if this is the correct number of bytes per pixel
			frame = new Frame(WIDTH, HEIGHT, 4);
			frame->format = Frame::BGRX;
		}

		void CopyIndividualChanelsToFrameLoop(unsigned char *R, unsigned char *G, unsigned char *B, int width, int height)
		{
			// iterate over each channel and copy it to the output frame in an interleaved format
			for(int row=0; row<height; row++)
			{
				if(row >= frame->height)
					// skip past this row as it is out of bounds
						continue;
				for(int col=0; col<width; col++)
				{
					if(col >= frame->width)
						continue;

					int destinationIndex = (row * width * 4) + (col * 4);
					frame->data[destinationIndex + 0] = B[(row * width) + col];
					frame->data[destinationIndex + 1] = G[(row * width) + col];
					frame->data[destinationIndex + 2] = R[(row * width) + col];
				}
			}
		}

		void CopyIndividualChanelsToFrame(unsigned char *R, unsigned char *G, unsigned char *B, int width, int height)
		{
			memcpy(frame->data, B, width * height * 4);
			//memcpy(&(frame->data[width * height]), G, width * height);
			//memcpy(&(frame->data[width * height * 2]), R, width * height);
		}

		void CopySurfaceDataToFrame(unsigned char* surfaceBuffer, int width, int height)
		{
			memcpy(frame->data, surfaceBuffer, width * height * 4);
		}

		bool initializeMediaSdk()
		{
			mfxStatus sts = MFX_ERR_NONE;

			m_fourcc = MFX_FOURCC_RGB4; //MFX_CODEC_JPEG;

			// no idea if this is needed, is this only for rendering?
			//m_memType = D3D11_MEMORY; // could also use D3D9_MEMORY which is default

			m_nMaxFps = 30;

			m_nFrames = MFX_INFINITE;

			// Initialize Intel Media SDK session
			// - MFX_IMPL_AUTO_ANY selects HW acceleration if available (on any adapter)
			// - Version 1.0 is selected for greatest backwards compatibility.
			// OS specific notes
			// - On Windows both SW and HW libraries may present
			// - On Linux only HW library only is available
			//   If more recent API features are needed, change the version accordingly
			m_implementation = MFX_IMPL_HARDWARE_ANY;//MFX_IMPL_HARDWARE_ANY;//MFX_IMPL_HARDWARE_ANY;//MFX_IMPL_AUTO_ANY;//MFX_IMPL_HARDWARE_ANY;
			mfxVersion ver = { {4, 1} };
			//ver.Major = 1;
			//ver.Minor = 11;

			if(useVideoMemory)
				sts = Initialize(m_implementation, ver, &m_mfxSession, &mfxAllocator);
			else
				sts = Initialize(m_implementation, ver, &m_mfxSession, NULL);
			MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

			// Create Media SDK decoder
			m_pmfxDEC = new MFXVideoDECODE(m_mfxSession);
			MSDK_CHECK_POINTER(m_pmfxDEC, MFX_ERR_MEMORY_ALLOC);
			//MFXVideoDECODE mfxDEC(session);

			// Set required video parameters for decode
			//memset(&m_mfxVideoParams, 0, sizeof(mfxVideoParam));
			//m_mfxVideoParams.mfx.CodecId = MFX_CODEC_JPEG;
			//m_mfxVideoParams.mfx.FrameInfo.FourCC = MFX_FOURCC_RGB4;
			//m_mfxVideoParams.mfx.FrameInfo.ChromaFormat = MFX_CHROMAFORMAT_YUV444;
			//m_mfxVideoParams.IOPattern = MFX_IOPATTERN_OUT_SYSTEM_MEMORY;//MFX_IOPATTERN_OUT_VIDEO_MEMORY; // when we switch to using D3D surfaces this will be MFX_IOPATTERN_OUT_VIDEO_MEMORY

			return true;
		}

		/*mfxStatus Initialize(mfxIMPL impl, mfxVersion ver, MFXVideoSession* pSession, mfxFrameAllocator* pmfxAllocator, bool bCreateSharedHandles = false)
		{
			mfxStatus sts = MFX_ERR_NONE;

#ifdef DX11_D3D
			impl |= MFX_IMPL_VIA_D3D11;
#endif

			// Initialize Intel Media SDK Session
			sts = pSession->Init(impl, &ver);
			MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

#if defined(DX9_D3D) || defined(DX11_D3D)
			// If mfxFrameAllocator is provided it means we need to setup DirectX device and memory allocator
			if (pmfxAllocator) {
				// Create DirectX device context
				mfxHDL deviceHandle;
				sts = CreateHWDevice(*pSession, &deviceHandle, NULL, bCreateSharedHandles);
				MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

				// Provide device manager to Media SDK
				sts = pSession->SetHandle(MFX_HANDLE_D3D11_DEVICE, deviceHandle);
				MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

				pmfxAllocator->pthis  = *pSession; // We use Media SDK session ID as the allocation identifier
				pmfxAllocator->Alloc  = simple_alloc;
				pmfxAllocator->Free   = simple_free;
				pmfxAllocator->Lock   = simple_lock;
				pmfxAllocator->Unlock = simple_unlock;
				pmfxAllocator->GetHDL = simple_gethdl;

				// Since we are using video memory we must provide Media SDK with an external allocator
				sts = pSession->SetFrameAllocator(pmfxAllocator);
				MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);
			}
#endif

			return sts;
		}*/

		bool decodeFirstHeader(unsigned char *buf, size_t len)
		{
			mfxStatus sts = MFX_ERR_NONE;

			memset(&m_mfxVideoParams, 0, sizeof(mfxVideoParam));
			m_mfxVideoParams.mfx.CodecId = MFX_CODEC_JPEG;
			if(useVideoMemory)
				m_mfxVideoParams.IOPattern = MFX_IOPATTERN_OUT_VIDEO_MEMORY; // when we switch to using D3D surfaces this will be MFX_IOPATTERN_OUT_VIDEO_MEMORY
			else
				m_mfxVideoParams.IOPattern = MFX_IOPATTERN_OUT_SYSTEM_MEMORY; // when we switch to using D3D surfaces this will be MFX_IOPATTERN_OUT_VIDEO_MEMORY

			memset(&m_mfxBS, 0, sizeof(m_mfxBS));
			m_mfxBS.MaxLength = len;
			m_mfxBS.Data = buf;
			m_mfxBS.DataLength = len;
			MSDK_CHECK_POINTER(m_mfxBS.Data, MFX_ERR_MEMORY_ALLOC);

			// RELEVANT FORUM POST HERE FOR GETTING THIS CODE WORKING WITH JPEG:
			// https://software.intel.com/en-us/articles/supported-output-color-formats-in-mjpeg-decoding
			// https://software.intel.com/en-us/forums/intel-media-sdk/topic/560053
			sts = m_pmfxDEC->DecodeHeader(&m_mfxBS, &m_mfxVideoParams);
			MSDK_IGNORE_MFX_STS(sts, MFX_WRN_PARTIAL_ACCELERATION);
			MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

			m_mfxVideoParams.mfx.FrameInfo.FourCC = MFX_FOURCC_RGB4;
			m_mfxVideoParams.mfx.FrameInfo.ChromaFormat = MFX_CHROMAFORMAT_YUV444;

			// query to find the actual API version supported for the implementation specified
			MFXQueryIMPL(m_mfxSession, &actualImplementation); // returns the actual implementation of the session
			MFXQueryVersion(m_mfxSession, &actualVersion);

			//---------------------------------------------
			// TODO: Does this need to be done for every frame or just the first one
			// Query number of required surfaces for decoder
			mfxFrameAllocResponse mfxResponse;
			mfxFrameAllocRequest Request;
			memset(&Request, 0, sizeof(Request));
			sts = m_pmfxDEC->QueryIOSurf(&m_mfxVideoParams, &Request);
			MSDK_IGNORE_MFX_STS(sts, MFX_WRN_PARTIAL_ACCELERATION);
			MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

			numSurfaces = Request.NumFrameSuggested;

			mfxU16 width = (mfxU16) MSDK_ALIGN32(Request.Info.Width);
			mfxU16 height = (mfxU16) MSDK_ALIGN32(Request.Info.Height);

			if(useVideoMemory)
			{
				Request.Type |= WILL_READ; // This line is only required for Windows DirectX11 to ensure that surfaces can be retrieved by the application

				// Allocate surfaces for decoder
				sts = mfxAllocator.Alloc(mfxAllocator.pthis, &Request, &mfxResponse);
				MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);
			}
			else
			{
				// Allocate surfaces for decoder
				// - Width and height of buffer must be aligned, a multiple of 32
				// - Frame surface array keeps pointers all surface planes and general frame info
				mfxU8 bitsPerPixel = 32;        // BGRX is a 32 bits per pixel format
				//!!!!!! Using system memory for allocation - NOT RECOMMENDED, since lowers performance
				// See the following for examples of using video memory https://software.intel.com/en-us/articles/framework-for-developing-applications-using-media-sdk
				surfaceSize = width * height * bitsPerPixel / 8; 
				surfaceBuffers = (mfxU8*) new mfxU8[surfaceSize * numSurfaces];
			}

			// Allocate surface headers (mfxFrameSurface1) for decoder
			pmfxSurfaces = new mfxFrameSurface1 *[numSurfaces];
			MSDK_CHECK_POINTER(pmfxSurfaces, MFX_ERR_MEMORY_ALLOC);
			for (int i = 0; i < numSurfaces; i++) {
				pmfxSurfaces[i] = new mfxFrameSurface1;
				memset(pmfxSurfaces[i], 0, sizeof(mfxFrameSurface1));
				memcpy(&(pmfxSurfaces[i]->Info), &(m_mfxVideoParams.mfx.FrameInfo), sizeof(mfxFrameInfo));

				if(useVideoMemory)
					// the following block of code is when using video memory
					pmfxSurfaces[i]->Data.MemId = mfxResponse.mids[i];      // MID (memory id) represents one video NV12 surface
				else
				{
					// the following block of code is for when using main memory
					pmfxSurfaces[i]->Data.B = &surfaceBuffers[surfaceSize * i];
					pmfxSurfaces[i]->Data.G = pmfxSurfaces[i]->Data.B + 1;
					pmfxSurfaces[i]->Data.R = pmfxSurfaces[i]->Data.B + 2;
					pmfxSurfaces[i]->Data.A = pmfxSurfaces[i]->Data.B + 3;
					pmfxSurfaces[i]->Data.Pitch = width*4;
				}
			}

			// Now that we have allocated the surfaces we can proceed and initialize the Media SDK decoder
			sts = m_pmfxDEC->Init(&m_mfxVideoParams);
			MSDK_IGNORE_MFX_STS(sts, MFX_WRN_PARTIAL_ACCELERATION);
			MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

			return true;
		}

		bool decompress(unsigned char *buf, size_t len)
		{
			mfxStatus sts = MFX_ERR_NONE;

			if(!decodedFirstHeader)
			{
				bool decodeHeaderSuccess = decodeFirstHeader(buf, len);
				decodedFirstHeader = true;
			}

			// zero out the Media SDK bit stream structure, set it to point to the jpg buffer from camera
			memset(&m_mfxBS, 0, sizeof(m_mfxBS));
			m_mfxBS.MaxLength = len;
			m_mfxBS.Data = buf;
			m_mfxBS.DataLength = len;
			MSDK_CHECK_POINTER(m_mfxBS.Data, MFX_ERR_MEMORY_ALLOC);

			// ===============================================================
			// Start decoding the frames
			//

			mfxTime tStart, tEnd;
			mfxGetTime(&tStart);

			mfxSyncPoint syncp;
			mfxFrameSurface1* pmfxOutSurface = NULL;
			int nIndex = 0;
			mfxU32 nFrame = 0;

			//
			// Stage 1: Main decoding loop.
			// NOTE: Should look into asynchronous call handling (GetFreeTaskIndex) to have multiple frames in parallel processing (this would
			//	     have to be done across multiple cameras as we are getting frames in realtime so can't speed up processing for a single camera
			//
			while (MFX_ERR_NONE <= sts || MFX_ERR_MORE_DATA == sts || MFX_ERR_MORE_SURFACE == sts) 
			{
				if (MFX_WRN_DEVICE_BUSY == sts)
					std::this_thread::sleep_for(std::chrono::milliseconds(1));  // Wait if device is busy, then repeat the same call to DecodeFrameAsync


				if (MFX_ERR_MORE_SURFACE == sts || MFX_ERR_NONE == sts) 
				{
					nIndex = GetFreeSurfaceIndex(pmfxSurfaces, numSurfaces);        // Find free frame surface
					MSDK_CHECK_ERROR(MFX_ERR_NOT_FOUND, nIndex, MFX_ERR_MEMORY_ALLOC);
				}
				// Decode a frame asychronously (returns immediately)
				//  - If input bitstream contains multiple frames DecodeFrameAsync will start decoding multiple frames, and remove them from bitstream
				sts = m_pmfxDEC->DecodeFrameAsync(&m_mfxBS, pmfxSurfaces[nIndex], &pmfxOutSurface, &syncp);

				// Ignore warnings if output is available,
				// if no output and no action required just repeat the DecodeFrameAsync call
				if (MFX_ERR_NONE < sts && syncp)
					sts = MFX_ERR_NONE;

				if (MFX_ERR_NONE == sts)
					sts = m_mfxSession.SyncOperation(syncp, 60000);      // Synchronize. Wait until decoded frame is ready

				if (MFX_ERR_NONE == sts) 
				{
					++nFrame;

					if(useVideoMemory)
					{
					sts = mfxAllocator.Lock(mfxAllocator.pthis, pmfxOutSurface->Data.MemId, &(pmfxOutSurface->Data));
					MSDK_BREAK_ON_ERROR(sts);

					mfxFrameData* pData = &pmfxOutSurface->Data;
					mfxFrameInfo* pInfo = &pmfxOutSurface->Info;

					CopyIndividualChanelsToFrame(pmfxOutSurface->Data.R, pmfxOutSurface->Data.G, pmfxOutSurface->Data.B, WIDTH, HEIGHT);
					//sts = WriteRawFrame(pmfxOutSurface, fSink);
					//MSDK_BREAK_ON_ERROR(sts);

					sts = mfxAllocator.Unlock(mfxAllocator.pthis, pmfxOutSurface->Data.MemId, &(pmfxOutSurface->Data));
					MSDK_BREAK_ON_ERROR(sts);
					}
					else
					{
						// the following block is for main memory cases
						mfxFrameData* pData = &pmfxOutSurface->Data;
						mfxFrameInfo* pInfo = &pmfxOutSurface->Info;

						// check that the resolution is what we expect
						if(pInfo->Width != WIDTH || (pInfo->Height != HEIGHT && pInfo->Height != 1088))
						{
							// thats strange, the resolution isn't what we expect, this class is designed to only handle
							// the Kinect V2 resolution images so doesn't generalise at present (just because of image buffers allocated with hard coded sizes).
							// The height of the image is often slightly larger than the actual height so this part needs
							// to be cropped off.
						}
						else
						{
							CopySurfaceDataToFrame((unsigned char*)&surfaceBuffers[nIndex * surfaceSize], WIDTH, HEIGHT);
						}
					}
					
					break;
				}

				if(MFX_ERR_MORE_DATA == sts)
					printf("Error: It wants more data but we should have a whole frames worth\n");
			}

			// MFX_ERR_MORE_DATA means that file has ended, need to go to buffering loop, exit in case of other errors
			MSDK_IGNORE_MFX_STS(sts, MFX_ERR_MORE_DATA);
			MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

			mfxGetTime(&tEnd);
			double elapsed = TimeDiffMsec(tEnd, tStart) / 1000;
			double fps = ((double)nFrame / elapsed);
			if((nFrame % 12) == 0)
				printf("\nExecution time: %3.2f s (%3.2f fps)\n", elapsed, fps);

			return true;
		}

		// Get free raw frame surface
		int GetFreeSurfaceIndex(mfxFrameSurface1** pSurfacesPool, mfxU16 nPoolSize)
		{
			if (pSurfacesPool)
				for (mfxU16 i = 0; i < nPoolSize; i++)
					if (0 == pSurfacesPool[i]->Data.Locked)
						return i;
			return MFX_ERR_NOT_FOUND;
		}
	};

	MediaSdkRgbPacketProcessor::MediaSdkRgbPacketProcessor() :
		impl_(new MediaSdkRgbPacketProcessorImpl())
	{
	}

	MediaSdkRgbPacketProcessor::~MediaSdkRgbPacketProcessor()
	{
		delete impl_;
	}

	bool MediaSdkRgbPacketProcessor::good()
	{
		return impl_->good;
	}

	void MediaSdkRgbPacketProcessor::process(const RgbPacket &packet)
	{
		if (listener_ == 0)
			return;

		impl_->startTiming();

		impl_->frame->timestamp = packet.timestamp;
		impl_->frame->sequence = packet.sequence;
		impl_->frame->exposure = packet.exposure;
		impl_->frame->gain = packet.gain;
		impl_->frame->gamma = packet.gamma;

		unsigned char *jpegData = packet.jpeg_buffer;
		size_t jpegDataLength = packet.jpeg_buffer_length;
		//VaapiBuffer *vb = static_cast<VaapiBuffer *>(packet.memory);
		impl_->good = impl_->decompress(jpegData, jpegDataLength);

		impl_->stopTiming(LOG_INFO);

		if (!impl_->good)
			impl_->frame->status = 1;

		if (listener_->onNewFrame(Frame::Color, impl_->frame))
			impl_->newFrame();
	}

} /* namespace libfreenect2 */

#endif