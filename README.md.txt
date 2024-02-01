How to Run:
Get data from kinect:
Run RunData.m, 
- modify:
	numberOfFrames : (200) number of frames to record,
	d : ("datasetA") name of dataset.
- saves kinect results to dataset folder

To Segment azure kinect data:
Run RunSegmentation.m, 
- modify:
	d : ("datasetA") name of dataset.
- saves results to dataset folder
To run DT, ICP, keypoints, prep mocap data

Run RunSource.m:
- modify:
	d : ("datasetA") name of dataset.
- saves results to running folder

To get graphs:
Run showAndSaveResults.m,
- modify: 
	d : ("datasetA") name of dataset.
- saves graphs to results folder.
