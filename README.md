# Eclipse-GroundStation-Laptop


---


The code contained in this repository is a modification of code provided by Louisiana State University for the ground station laptop used in the Eclipse Ballooning Project. The main project page for more information can be found [here](http://eclipse.montana.edu/).


---












Here is the line of code that turns a .ui file that is output from QT Creator into a .py file. This example turns the GTS_mainwindow_vL1_0.ui file into its corresponding .py file.
```python
pyuic -x GTS_mainwindow_vL1_0.ui -o GTS_mainwindow_vL1_0.py -x
```
