how to use

Eye in hand calibration
`ros launch sagittarius_object_color_detector camera_calibration_hsv.launch`
The parameters of line-limited regression are saved in `vision_config.yaml`

HSV color range settings
`ros launch sagittarius_object_color_detector hsv_params.launch`
The hsv parameters of the four colors are saved in `vision_config.yaml`

Pick up once, the color of the grabbed block is blue by default
`ros launch sagittarius_object_color_detector object_pick.launch`

Routine for color classification, placement is fixed
`ros launch sagittarius_object_color_detector color_classification_fixed.launch`
