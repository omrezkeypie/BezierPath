# ABOUT
BezierPath is a flexible bezier spline module, designed to be easy to implenent and use while being optimizied for large scale uses.

## API

* BezierPath.new(Positions)
Takes in a table of positions which will be used as the waypoints for the spline and returns the path object.

* Path:CalculateUniformCFrame(T)
Takes in a T value from 0-1 and returns a uniform cframe across the path object.

* Path:CalculateUniformPosition(T)
Takes in a T value from 0-1 and returns a uniform position across the path object.
