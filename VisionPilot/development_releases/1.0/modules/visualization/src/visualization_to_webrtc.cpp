#include <visualization/visualization_to_webrtc.hpp>

// GStreamer headers for WebRTC streaming
#include <gst/app/gstappsrc.h>
#include <gst/sdp/gstsdpmessage.h>
#include <gst/webrtc/webrtc.h>

// libsoup for WebSocket signaling
#include <libsoup/soup.h>

// JSON-GLib for JSON handling in signaling messages
#include <json-glib/json-glib.h>

#include <algorithm>
#include <cmath>
#include <string>


namespace visualization {


    namespace {


        // Helper func to initialize GStreamer once (thread-safe)
        void init_gstreamer_once() {

            static std::once_flag once;
            std::call_once(once, []() {
                gst_init(nullptr, nullptr);
            });

        }


        // Helper func to escape JSON strings for signaling messages
        std::string escape_json(
            const std::string & value
        ) {
            gchar * escaped = g_strescape(value.c_str(), nullptr);
            std::string result = escaped != nullptr ? escaped : "";
            g_free(escaped);

            return result;
        };


        // Helper func to generate JSON signaling message for SDP offer
        std::string make_offer_message(
            const std::string & sdp_offer
        ) {

            return  std::string{
                        "{ \"type\": \"offer\", \"sdp\": \""
                    } +                                                 \
                    escape_json(sdp_offer) + "\" }";

        };


        // Helper func to generate JSON signaling message for ICE candidate
        std::string make_candidate_message(
            int sdp_mline_index,
            const std::string & candidate
        ) {

            return  std::string{
                    "{ \"type\": \"candidate\", \"sdpMLineIndex\": "
                    } +                                                 \
                    std::to_string(sdp_mline_index) +                   \
                    ", \"candidate\": \"" +                             \
                    escape_json(candidate) +                            \
                    "\" }";

        };


        // Helper func to ensure all received frames are in BGR format for WebRTC streaming
        cv::Mat ensure_bgr_frame(
            const cv::Mat & frame
        ) {

            // Return empty frame as is
            if (frame.empty()) {
                return frame;
            };

            // If already in BGR format, return as is (or clone if not continuous)
            if (frame.type() == CV_8UC3) {
                return frame.isContinuous() ? frame : frame.clone();
            };

            // Convert grayscale or BGRA frames to BGR format for WebRTC streaming
            cv::Mat converted;
            if (frame.type() == CV_8UC1) {          // If grayscale
                cv::cvtColor(
                    frame, 
                    converted, 
                    cv::COLOR_GRAY2BGR
                );
            } else if (frame.type() == CV_8UC4) {   // If BGRA
                cv::cvtColor(
                    frame,
                    converted,
                    cv::COLOR_BGRA2BGR
                );
            } else {                                // For other formats, try direct conversion
                frame.convertTo(
                    converted,
                    CV_8UC3
                );
                if (converted.channels() != 3) {
                    return cv::Mat();
                };
            };

        };


    };

}