import cv2
import numpy as np

# Mouse click callback function
def fill_region(event, x, y, flags, param):
    global img, resized_img  # Declare both img and resized_img as global variables
    if event == cv2.EVENT_LBUTTONDOWN:
        # Convert coordinates from resized image to original image
        x_original = int(x * (img.shape[1] / resized_img.shape[1]))
        y_original = int(y * (img.shape[0] / resized_img.shape[0]))

        # Use flood-fill algorithm to change the black region to white
        cv2.floodFill(img, None, (x_original, y_original), 255)

        # Update resized image to reflect changes
        resized_img = cv2.resize(img, (new_width, new_height))


# Read image
img = cv2.imread("maps/map.png", cv2.IMREAD_GRAYSCALE)

# Resize image to fit into a smaller window (adjust dimensions as needed)
new_height = 1000
new_width = int(img.shape[1] * (new_height / img.shape[0]))
resized_img = cv2.resize(img, (new_width, new_height))

#make everything white
# for i in range(0, resized_img.shape[0]):
#     for j in range(0, resized_img.shape[1]):
#         resized_img[i][j] = 255
# # save
# cv2.imwrite("maps/white_map.png", resized_img)

# Create a window
cv2.namedWindow('image')
cv2.setMouseCallback('image', fill_region)

while True:
    # Show image
    cv2.imshow('image', resized_img)

    # Wait for key press and check if it's 'Esc' or 's'
    key = cv2.waitKey(20) & 0xFF

    # Exit when 'Esc' key is pressed
    if key == 27:
        break

    # Save the image when 's' key is pressed
    if key == ord('s'):
        cv2.imwrite("modified_image.png", img)

cv2.destroyAllWindows()
