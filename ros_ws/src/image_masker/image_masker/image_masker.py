import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError # Import CvBridgeError for specific exception handling
import cv2
import os
import torch
import numpy as np
import message_filters
from ament_index_python.packages import get_package_share_directory
import threading

class ImageMasker(Node): # Renamed class
    """
    A ROS2 node that subscribes to left and right rectified stereo image topics,
    performs real-time person masking using YOLO segmentation, and publishes the masked images.
    Handles both grayscale and color input images.
    """
    def __init__(self):
        super().__init__('image_masker_node') # Consistent node name

        # Get package share directory for robust file path resolution
        share_dir = get_package_share_directory('image_masker') # Assuming package name is 'image_masker'
        package_file = os.path.join(share_dir, 'model', 'yolo11n-seg.pt') # Recommend yolov8n-seg.pt

        # Declare parameters for the YOLO model path, person class ID, and publish flag
        self.declare_parameter('yolo_model_path', package_file)
        self.declare_parameter('person_class_id', 0) # COCO dataset ID for 'person' is 0
        self.declare_parameter('publish_masked_images', True)

        self.br = CvBridge()

        yolo_model_to_load = self.get_parameter('yolo_model_path').get_parameter_value().string_value
        try:
            self.model = YOLO(yolo_model_to_load)
            self.get_logger().info(f"Successfully loaded YOLO segmentation model from: {yolo_model_to_load}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {yolo_model_to_load}. Error: {e}")
            rclpy.shutdown()
            return

        self.person_class_id = self.get_parameter('person_class_id').get_parameter_value().integer_value
        self.publish_masked_images = self.get_parameter('publish_masked_images').get_parameter_value().bool_value

        if self.publish_masked_images:
            # Publishers will use 'passthrough' encoding to match input
            self.masked_left_image_publisher = self.create_publisher(Image, '/stereo/left/rect/masked', 10)
            self.masked_right_image_publisher = self.create_publisher(Image, '/stereo/right/rect/masked', 10)
            self.get_logger().info('Publishing masked images to /stereo/left/rect/masked and /stereo/right/rect/masked')

        self.left_sub = message_filters.Subscriber(self, Image, '/stereo/left/rectified_images')
        self.right_sub = message_filters.Subscriber(self, Image, '/stereo/right/rectified_images')
    
        self.ts = message_filters.ApproximateTimeSynchronizer([self.left_sub, self.right_sub], 10, 0.1)
        self.ts.registerCallback(self.stereo_image_callback)

        self.get_logger().info('ImageMasker node has been started and is ready for image synchronization.') # Updated log
        self.get_logger().info('Subscribing to /stereo/left/rectified_images and /stereo/right/rectified_images topics.')

    def _process_single_image_thread(self, cv_image, original_encoding, result_container):
        """
        Helper method to process a single image with YOLO and store the result.
        Designed to be run in a separate thread.
        Takes the original OpenCV image and its encoding.
        """
        try:
            masked_image = self.process_image_with_yolo(cv_image, original_encoding)
            result_container['image'] = masked_image
        except Exception as e:
            self.get_logger().error(f"Error processing image in thread: {e}")
            result_container['image'] = None # Indicate failure

    def stereo_image_callback(self, left_msg, right_msg):
        """
        Callback function for synchronized left and right rectified image topics.
        Performs YOLO segmentation and masks out persons using threading.
        """
        self.get_logger().debug('Received synchronized stereo images.')

        left_cv_image = None
        right_cv_image = None
        
        # Store original encodings to publish back in the same format
        left_original_encoding = left_msg.encoding
        right_original_encoding = right_msg.encoding

        try:
            # Convert ROS Image messages to OpenCV images (numpy arrays)
            # Use 'passthrough' to let cv_bridge handle the conversion based on message's actual encoding
            left_cv_image = self.br.imgmsg_to_cv2(left_msg, desired_encoding='passthrough')
            right_cv_image = self.br.imgmsg_to_cv2(right_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error during image conversion: {e}")
            return

        # Containers to hold results from threads
        left_result = {'image': None}
        right_result = {'image': None}

        # Create threads for processing. Pass original image and its encoding.
        left_thread = threading.Thread(target=self._process_single_image_thread,
                                       args=(left_cv_image, left_original_encoding, left_result))
        right_thread = threading.Thread(target=self._process_single_image_thread,
                                        args=(right_cv_image, right_original_encoding, right_result))

        # Start the threads
        left_thread.start()
        right_thread.start()

        # Wait for both threads to complete
        left_thread.join()
        right_thread.join()

        masked_left_image = left_result['image']
        masked_right_image = right_result['image']

        # Check if processing was successful for both images
        if masked_left_image is None or masked_right_image is None:
            self.get_logger().error("One or both images failed processing in threads. Skipping publish.")
            return

        if self.publish_masked_images:
            try:
                # Convert processed OpenCV images back to ROS Image messages
                # Use the original encoding for the output topic
                masked_left_ros_image = self.br.cv2_to_imgmsg(masked_left_image, encoding=left_original_encoding)
                masked_left_ros_image.header = left_msg.header # Keep the original timestamp and frame_id
                self.masked_left_image_publisher.publish(masked_left_ros_image)

                masked_right_ros_image = self.br.cv2_to_imgmsg(masked_right_image, encoding=right_original_encoding)
                masked_right_ros_image.header = right_msg.header # Keep the original timestamp and frame_id
                self.masked_right_image_publisher.publish(masked_right_ros_image)

                self.get_logger().debug('Published masked stereo images.')
            except CvBridgeError as e: # Catch CvBridgeError specifically for publishing
                self.get_logger().error(f"Image publishing CvBridge Error: {e}")
            except Exception as e: # Catch other exceptions during publishing
                self.get_logger().error(f"Image publishing general Error: {e}")

    def process_image_with_yolo(self, cv_image, original_encoding):
        """
        Runs YOLO inference on an OpenCV image, finds person masks,
        and replaces the segmented person areas with black pixels.
        Adapts to original image encoding for input to YOLO and final output.
        """
        # Determine the number of channels in the input OpenCV image
        # cv_image.shape will be (H, W) for grayscale, (H, W, C) for color
        num_channels = 1 if len(cv_image.shape) == 2 else cv_image.shape[2]

        # Prepare image for YOLO: YOLO models typically expect 3-channel BGR/RGB input.
        cv_image_for_yolo = None
        if num_channels == 1: # If input is grayscale (1 channel)
            cv_image_for_yolo = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        elif num_channels == 3: # If input is already 3-channel (BGR)
            cv_image_for_yolo = cv_image
        else:
            self.get_logger().error(f"Unsupported number of channels in input image: {num_channels}. Expected 1 or 3.")
            # Return a black image of the original shape to prevent further errors
            return np.zeros_like(cv_image)

        # Ensure the image for YOLO is contiguous, which can sometimes help with compatibility
        cv_image_contiguous_for_yolo = np.ascontiguousarray(cv_image_for_yolo)

        # Run YOLO inference. 'verbose=False' reduces console output.
        results = self.model(cv_image_contiguous_for_yolo, verbose=False)

        # Initialize an empty mask that will accumulate all person masks.
        # This mask must have the same dimensions as the original input image.
        combined_person_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)

        # Iterate through the results for the current image
        for r in results:
            # Ensure masks and boxes are present for the current result object
            if r.masks is not None and r.boxes is not None:
                masks_data = r.masks.data # Raw mask data as a torch.Tensor (N, H_mask, W_mask)
                class_ids = r.boxes.cls # Class IDs for each detected object as a torch.Tensor

                # Find indices where the class ID corresponds to 'person' (COCO class ID 0)
                person_indices = torch.where(class_ids == self.person_class_id)[0]

                if person_indices.numel() > 0: # Check if any persons were detected
                    # Extract only the masks corresponding to persons
                    person_masks_tensors = masks_data[person_indices] # (Num_persons, H_mask, W_mask)

                    # Combine all individual person masks into a single binary mask.
                    # torch.any(dim=0) creates a mask where a pixel is True if it's True in *any* of the input masks.
                    # .int() converts True/False to 1/0, then multiply by 255 for OpenCV mask format.
                    combined_person_mask_tensor = torch.any(person_masks_tensors, dim=0).int() * 255
                    
                    # Convert the combined mask tensor to a NumPy array.
                    mask_np = combined_person_mask_tensor.cpu().numpy().astype(np.uint8)

                    # --- CRITICAL STEP: RESIZE MASK TO MATCH ORIGINAL IMAGE DIMENSIONS ---
                    # Ensure the mask has the exact same (height, width) as the original image.
                    if mask_np.shape[:2] != cv_image.shape[:2]:
                        self.get_logger().warn(f"YOLO mask shape {mask_np.shape[:2]} does not match original image shape {cv_image.shape[:2]}. Resizing mask using INTER_NEAREST.")
                        # cv2.resize expects (width, height) tuple
                        mask_np = cv2.resize(mask_np, (cv_image.shape[1], cv_image.shape[0]), interpolation=cv2.INTER_NEAREST)
                    
                    # Accumulate the current person mask into the overall combined_person_mask.
                    combined_person_mask = cv2.bitwise_or(combined_person_mask, mask_np)


        # Now, `combined_person_mask` is a binary mask (0 or 255) where 255 indicates a person.
        # To remove the person, we want to keep everything *except* the person, so we invert this mask.
        inverted_mask = cv2.bitwise_not(combined_person_mask)

        # Apply the inverted mask to the original input image.
        # The output image should have the same number of channels as the original input.
        if num_channels == 1: # If original was grayscale (1 channel)
            masked_image = cv2.bitwise_and(cv_image, inverted_mask)
        else: # If original was 3-channel (BGR/RGB)
            # Convert the single-channel inverted_mask to a 3-channel mask
            inverted_mask_3_channels = cv2.cvtColor(inverted_mask, cv2.COLOR_GRAY2BGR)
            masked_image = cv2.bitwise_and(cv_image, inverted_mask_3_channels)
        
        return masked_image


def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    image_masker_node = ImageMasker() # Use the consistent class name
    # Use a MultiThreadedExecutor to allow the node to process callbacks concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(image_masker_node, executor=executor)
    
    # Destroy the node once rclpy.spin() returns (e.g., on shutdown)
    image_masker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()