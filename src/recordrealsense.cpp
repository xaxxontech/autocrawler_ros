/*
rosrun autocrawler recordrs --audio-device=1 --video-width=1280 --video-height=720 --video-bitrate=256 --record-path=blob.3gp image_raw:=/camera/color/image_raw
* 
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <gst/gst.h>
#include <gst/app/app.h>
#include <string.h>


namespace enc = sensor_msgs::image_encodings;

static GstElement *pipe1, *appsrc_;
std::unique_ptr<image_transport::ImageTransport> image_transport_;
image_transport::Subscriber image_sub_;
static const gchar *audio_device = nullptr;
static const gchar *video_width = nullptr; 
static const gchar *video_height = nullptr; 
static const gchar *video_bitrate = nullptr; 
static const gchar *record_path = nullptr; 
static gboolean started = FALSE;

static GOptionEntry entries[] =
{
  { "audio-device", 0, 0, G_OPTION_ARG_STRING, &audio_device, "String of the audio device number", "ID" },
  { "video-width", 0, 0, G_OPTION_ARG_STRING, &video_width, "video width pixels", NULL },
  { "video-height", 0, 0, G_OPTION_ARG_STRING, &video_height, "video height pixels", NULL },
  { "video-bitrate", 0, 0, G_OPTION_ARG_STRING, &video_bitrate, "target bitrate kbps", NULL },
  { "record-path", 0, 0, G_OPTION_ARG_STRING, &record_path, "target path + filame", NULL },
  { NULL },
};


static gboolean configure_pipeline (void)
{
	GstStateChangeReturn ret;
	GError *error = NULL;
	
	// plays in chrome desktop, not android, shows error in firefox	
	// gchar *pl = g_strconcat (
		// "videorate ! video/x-raw,width=", video_width, ",height=", video_height, ",framerate=15/1 ! "
		// "videoconvert ! queue ! x264enc tune=zerolatency bitrate=512 ! "
		// "mux. alsasrc device=hw:", audio_device, " ! "
		// "queue ! audioconvert ! audioresample quality=2 ! voaacenc ! aacparse ! qtmux name=mux ! "
		// "filesink location=", record_path, " sync=false"
		// , NULL);
		
	// better, plays in chrome desktop, not android, forces download instead of error in firefox	
	gchar *pl = g_strconcat (
		"videorate ! video/x-raw,width=", video_width, ",height=", video_height, ",framerate=15/1 ! "
		"videoconvert ! queue ! x264enc tune=zerolatency bitrate=512 ! "
		"mux. alsasrc device=hw:", audio_device, " ! "
		"queue ! audioconvert ! audioresample quality=2 ! voaacenc tolerance=9223372036854775807 ! aacparse ! 3gppmux name=mux ! "
		"filesink location=", record_path, " sync=false"
		, NULL);
		
	// gst-launch-1.0 -e v4l2src device="/dev/video0" ! videoconvert ! queue ! x264enc tune=zerolatency ! mux. alsasrc device="hw:2" 
	// ! queue ! audioconvert ! audioresample ! voaacenc ! aacparse ! qtmux name=mux ! filesink location=test.mp4 sync=false
	
	pipe1 =	gst_parse_launch (pl, &error);
	g_free(pl);

	if (error) {
		g_printerr ("Failed to parse launch: %s\n", error->message);
		g_error_free (error);
		if (pipe1)  g_clear_object (&pipe1);
		return FALSE;
	}
	
  
  	// init appsrc
  	appsrc_ = gst_element_factory_make("appsrc", "source");
	if (appsrc_ == nullptr) {
		ROS_ERROR("GST: failed to create appsrc!");
		return FALSE;
	}

	gst_app_src_set_stream_type(GST_APP_SRC_CAST(appsrc_), GST_APP_STREAM_TYPE_STREAM);
	gst_app_src_set_latency(GST_APP_SRC_CAST(appsrc_), 0, -1);
	g_object_set(GST_OBJECT(appsrc_),
			"format", GST_FORMAT_TIME,
			"is-live", true,
			"max-bytes", 0,
			"do-timestamp", true,
			NULL);

	// find pipeline sink (where we may link appsrc)
	GstPad *inpad = gst_bin_find_unlinked_pad(GST_BIN(pipe1), GST_PAD_SINK);
	g_assert(inpad);

	GstElement *inelement = gst_pad_get_parent_element(inpad);
	g_assert(inelement);
	gst_object_unref(GST_OBJECT(inpad));

	if (!gst_bin_add(GST_BIN(pipe1), appsrc_)) {
		ROS_ERROR("GST: gst_bin_add() failed!");
		gst_object_unref(GST_OBJECT(pipe1));
		gst_object_unref(GST_OBJECT(inelement));
		return FALSE;
	}

	if (!gst_element_link(appsrc_, inelement)) {
		ROS_ERROR("GST: cannot link %s -> %s",
		gst_element_get_name(appsrc_),
		gst_element_get_name(inelement));
		gst_object_unref(GST_OBJECT(pipe1));
		gst_object_unref(GST_OBJECT(inelement));
		return FALSE;
	}

	gst_object_unref(GST_OBJECT(inelement));	
	
	// pause pipeline
	ret = gst_element_set_state(pipe1, GST_STATE_PAUSED);
	if (ret== GST_STATE_CHANGE_FAILURE) {
		ROS_ERROR("GST: state change error. Check pipeline.");
		return false;
	}

	return TRUE;
	
}


static gboolean start_pipeline (void) {
	GstStateChangeReturn ret;

	ROS_INFO ("Starting pipeline\n");
	ret = gst_element_set_state (GST_ELEMENT (pipe1), GST_STATE_PLAYING);
	if (ret == GST_STATE_CHANGE_FAILURE) {
			ROS_ERROR ("ERROR starting pipeline\n");
			if (pipe1)  g_clear_object (&pipe1);
			return FALSE;
	}
		
	started = TRUE;
	return TRUE;

}


GstCaps* gst_caps_new_from_image(const sensor_msgs::Image::ConstPtr &msg)
{
	// http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
	static const ros::M_string known_formats = {{
		{enc::RGB8, "RGB"},
		{enc::RGB16, "RGB16"},
		{enc::RGBA8, "RGBA"},
		{enc::RGBA16, "RGBA16"},
		{enc::BGR8, "BGR"},
		{enc::BGR16, "BGR16"},
		{enc::BGRA8, "BGRA"},
		{enc::BGRA16, "BGRA16"},
		{enc::MONO8, "GRAY8"},
		{enc::MONO16, "GRAY16_LE"},
	}};

	if (msg->is_bigendian) {
		ROS_ERROR("GST: big endian image format is not supported");
		return nullptr;
	}

	auto format = known_formats.find(msg->encoding);
	if (format == known_formats.end()) {
		ROS_ERROR("GST: image format '%s' unknown", msg->encoding.c_str());
		return nullptr;
	}

	return gst_caps_new_simple("video/x-raw",
			"format", G_TYPE_STRING, format->second.c_str(),
			"width", G_TYPE_INT, msg->width,
			"height", G_TYPE_INT, msg->height,
			"framerate", GST_TYPE_FRACTION, 0, 1,	// 0/1 = dynamic
			nullptr);
}


GstSample* gst_sample_new_from_image(const sensor_msgs::Image::ConstPtr &msg)
{
	auto buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
	g_assert(buffer);

	gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());
	GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_LIVE);

	auto caps = gst_caps_new_from_image(msg);
	if (caps == nullptr) {
		gst_object_unref(GST_OBJECT(buffer));
		return nullptr;
	}

	auto sample = gst_sample_new(buffer, caps, nullptr, nullptr);
	gst_buffer_unref(buffer);
	gst_caps_unref(caps);

	return sample;
}

void image_cb(const sensor_msgs::Image::ConstPtr &msg)
{
	// ROS_INFO("Image: %d x %d, stamp %f", msg->width, msg->height, msg->header.stamp.toSec());
	
	auto sample = gst_sample_new_from_image(msg);
	if (sample == nullptr)
		return;
		
	auto push_ret = gst_app_src_push_sample(GST_APP_SRC_CAST(appsrc_), sample);
	gst_sample_unref(sample);
	
	if (!started) start_pipeline();
}


int main(int argc, char **argv) 
{
	// added
	ros::init(argc, argv, "recordrs");
	ros::NodeHandle n;
	
	image_transport_.reset(new image_transport::ImageTransport(n));
	ROS_INFO("INIT recordrs");

	image_sub_ = image_transport_->subscribe("image_raw", 10, image_cb); 
	
	GOptionContext *context;
	GError *error = NULL;

	context = g_option_context_new ("- gstreamer recordrs");
	g_option_context_add_main_entries (context, entries, NULL);
	g_option_context_add_group (context, gst_init_get_option_group ());
	if (!g_option_context_parse (context, &argc, &argv, &error)) {
		g_printerr ("Error initializing: %s\n", error->message);
		return -1;
	}
	
	if (!configure_pipeline()) return -1;
	
	ros::Rate r(100); // 100 hz
	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}

	// cleanup
	if (pipe1) {
		gst_element_send_event(GST_ELEMENT (pipe1), gst_event_new_eos()); // required to properly close output file on CTRL-C
		ros::Duration(1.0).sleep();
		gst_element_set_state (GST_ELEMENT (pipe1), GST_STATE_NULL);
		g_print ("Pipeline stopped\n");
		gst_object_unref (pipe1);
	} else g_print("error no pipe1");

	return 0;
		
}
