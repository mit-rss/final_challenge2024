import cv2
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########

	bounding_box = (0,0,0,0)
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# h,w = hsv_img.shape[:2] #mask top and bottom
	# cv2.rectangle(hsv_img,(0,0),(w,int(h*.4)),(0,0,0),-1) #top rectangle
	# cv2.rectangle(hsv_img, (0, h-int(h*.4)), (w, h), (0, 0, 0), -1)#bottom rectangle


	# cv2.imwrite("hsv.jpg", hsv_img) 

	# red color bounds in HSV
	# Hue has values from 0 to 180, Saturation and Value from 0 to 255
	#red_low = (0, 100, 100)
	#red_high = (18, 255, 255)	

	#basically kept this the same, will adjust
	red_low1 = (-10, 168, 168)
	red_high1 = (10, 255, 255)	

	lower_red2 = (170, 100, 100)  # Adjusted lower range for wrapped values
	upper_red2 = (180, 255, 255)  # Adjusted upper range for wrapped values

	mask1 = cv2.inRange(hsv_img, red_low1, red_high1)
	mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)

	combined_mask = cv2.bitwise_or(mask1, mask2)


	# cv2.imwrite("mask.jpg", mask) 

	# erode and dilate to get rid of noise
	# structuring element is what erosion looks at - if everything inside is red, then it will be kept
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
	combined_mask = cv2.erode(combined_mask, kernel, iterations=1)
	# cv2.imwrite("erode.jpg", mask) 
	combined_mask = cv2.dilate(combined_mask, kernel, iterations=2)
	# cv2.imwrite("dilate.jpg", mask) 

	# returns list of contours and hiearchy, retr ignores inside countours, approx is for compression
	contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if len(contours) != 0:
		biggest_contour = max(contours, key = cv2.contourArea)
		x, y, w, h = cv2.boundingRect(biggest_contour)
		# bounding_box = ((x, y), (x + w, y + h))
  
		#modify to match that of stop light detector
		bounding_box = (x, y, x+w, y+h)
	else:
		#comment out later?
		print("Light not found.")

	# cv2.rectangle(img, bounding_box[0], bounding_box[1], (0, 0, 255), 2)
	# cv2.imshow('Contours', img)
	# cv2.imwrite("debug.jpg", img) 

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box
