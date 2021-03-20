macro(CopyPythonVisualizer SAMPLE_NAME)
	CopyFile(${SAMPLE_NAME}   VisualizeResult.py   VisualizeResult.py)
	CopyFile(${SAMPLE_NAME}   ../Utils/Visualizer.py   Visualizer.py)
endmacro()
