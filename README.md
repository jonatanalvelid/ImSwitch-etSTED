***This self-standing codebase is based on a previous version of ImSwitch. We are working towards implementing the etSTED widget and controller, and additional changes necessary for event-triggered acquisitions, in the main ImSwitch repository and latest version of ImSwitch (1.2) with its current developments.***

# ImSwitch-etSTED
 
This is a version of the control software ImSwitch (https://github.com/kasasxav/ImSwitch) from TestaLab (www.testalab.org) that integrates control of two imaging techniques in the smartSTED widget for performing event-triggered STED imaging, as used in the manuscript:
"Event-triggered STED imaging", Jonatan Alvelid, Martina Damenti, Ilaria Testa (manuscript, submitted, 2021)

The widget is available in view\widgets\etstedwidget.py, and the associated controller is available in controller\controllers\etstedcontroller.py.

The optimised real-time BAPTA calcium spike event detection analysis pipeline is available in etsted\analysis_pipelines\bapta_calcium_spikes.py (GPU-version) and bapta_calcium_spikes_cpu.py (CPU-version).

The coordinate transforms are available in etsted\transform_pipelines\coord_transform.py (general, for calibration during running ImSwitch) and wf_800_scan_80.py (pre-calibrated for the microscope used in the manuscript). 

## Installation
The software was tested to run on Windows 10 and with python=3.7. Typical install time for the environment in conda is 3-5 min.

Mock etSTED experiments can be performed with the simulated camera provided in ImSwitch-etSTED. For real etSTED experiments, the software requires a NI-DAQ data acquisition board and surrounding microscope hardware (see ImSwitch documentation for further information).

To prepare an environment, install ImSwitch from PyPI with the following command:
```
pip install ImSwitch
```

To run the optimised real-time BAPTA calcium spike event detection analysis pipeline additionally opencv has to be installed in the same environment, use the following command:
```
pip install opencv-python
```

In order to run the GPU-boosted analysis pipeline, CUDA Toolkit has additionally to be installed on the machine, together with the cupy package in the same environment. See instructions at https://docs.cupy.dev/en/stable/install.html. 

Choose the configuration file for your microscope in \config_files\options.json.

Start ImSwitch by running the ``` __main__.py ``` file.


## Demo - mock etSTED experiment
The default configuration file is etsted_sim.json, which contains a mock camera generating noisy images with occasional intensity spikes running at 10 Hz, a mock point-detector, and two mock lasers for performing mock etSTED experiments. The following steps can be followed to initiate a mock experiment, taking 1-3 min to set up and run:

1. Start LIVEVIEW of the mock camera by pressing ```LIVEVIEW``` in the ```Image Controls``` widget.
2. Switch to the etSTED widget by pressing the etSTED tab above the ```Laser Control``` widget.
3. Record a binary mask containing all the pixels by pressing ```Record binary mask```. 
4. Load a CPU-version of the bapta calcium spike detection pipeline by pressing ```Load pipeline```. 
5. Tick ```Visualize``` in order to run the mock experiment while showing the preprocessed images in real-time in the pop-out help widget.
6. Run mock experiment by pressing ```Initiate```.
7. The real-time red crosses on the image in the ```Image Display``` widget show detected events and spots where the STED scanning would have taken place. The mock camera returns an image with at most 1 peak at the same time, and as such at most 1 cross will be displayed at the same time.

If the widget softlocks due to not following the steps above and not having hardware connected, press the ```Unlock softlock``` button. 

## Running etSTED experiments
To perform etSTED experiments, use at least one camera for the fast method, one laser for the fast method, one laser for the scanning method, and one point-detector for the scanning method. Compile a configuration JSON file according to the instructions in the ImSwitch documentation and looking at the provided examples. The etSTED widget section requires the above-mentioned pieces of hardware to be listed. Compile an analysis pipeline for your task at hand, using the same input and output as the ones provided and additional numerical input parameters, alternatively use the intensity spike detection pipeline provided. Then follow the steps described above, with the following modifications:

1. Configure the camera settings in the ```Image Controls``` widget. Use an exposure time smaller than the ```Update period``` set in the etSTED widget.
2. Adjust the binary threshold and smoothing until a binary mask of the region of interest is calculated. This will automatically be used by the method upon recording. 
3. Choose an analysis pipeline in the dropdown menu, which displays all the ones in the etsted\analysis_pipelines\ folder, and load it by pressing ```Load pipeline```.
4. Perform a coordinate transform between the fast image space and the scanning image space by first recording the same area of a sample in both techniques (see ImSwitch documentation), and loading these in the ```Transform calibration``` help widget. Mark the coordinates of the same objects in the two images (at least 10), and press ```Save calibration```. Choose ```coord_transform``` in the dropdown menu to use the calibrated transform. 
5. Use the visualization mode described above while adjusting the pipeline parameters of the chosen analysis pipeline until the true events are detected in the best way possible. 
6. Set the scanning parameters (size, pixel size, lasers, and pixel dwell time) of the triggered scans in the scanning widget (see ImSwitch documenation). 
7. Untick ```Visualize```, and run the etSTED experiment by pressing ```Initiate```. Tick ```Endless``` if you want to run an endless experiment, repeating the process after each detected triggering event. 
8. All data and images are saved in the folder provided in the ```Recording``` widget (see ImSwitch documentation). 
