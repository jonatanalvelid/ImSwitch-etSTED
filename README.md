***This self-standing codebase is based on a previous version of ImSwitch and is currently not compatible with the latest version. The etSTED widget and controller, and additional changes necessary for event-triggered acquisitions, will be implemented in the latest version of ImSwitch (1.2) soon.***

# ImSwitch-etSTED
 
This is a version of the control software ImSwitch (https://github.com/kasasxav/ImSwitch) from TestaLab (www.testalab.org) that integrates control of two imaging techniques in the smartSTED widget for performing event-triggered STED imaging, as used in the manuscript:
"Event-triggered STED imaging", Jonatan Alvelid, Martina Damenti, Ilaria Testa (manuscript, submitted, 2021)

The widget is available in view\widgets\smartstedwidget.py, and the associated controller is available in controller\controllers\smartstedcontroller.py.

The optimised real-time analysis pipeline is available in smartsted\analysis_pipelines\bapta_calcium_spikes.py (GPU-version) and bapta_calcium_spikes_cpu.py (CPU-version).

The coordinate transforms are available in smartsted\transform_pipelines\coord_transform.py (general, for calibration during running ImSwitch) and wf_800_scan_80.py (pre-calibrated for the microscope used in the manuscript). 

## Installation
Run in an environment where you have installed imswitch through pip:
```
pip install ImSwitch
```
