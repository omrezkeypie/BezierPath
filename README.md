# ABOUT
BezierPath is an easy to use bezier spline module, designed to be used for TD games and general paths while being optimizied for large scale uses.

__NOTE__: BezierPath is NOT a traditional bezier spline. it behaves more like a linear spline that curves around each point.

## Example

A code snippet showing how to set up and use BezierPath.
```lua
local BezierPath = "Path to the module"
local Positions = {
    Vector3.new(0,10,0),
    Vector3.new(10,0,0),
    Vector3.new(10,10,10)
}
local NewPath = BezierPath.new(Positions,3)
local Part = Instance.new("Part",workspace)
part.Size = Vector3.new(1,1,1)
part.Anchored = true

for t = 0,1,1/100 do
    Part.CFrame = NewPath:CalculateUniformCFrame(t)
    task.wait(0.1)
end
```

## API

* BezierPath.new(Positions,CurveSize)

Takes in a table of positions which will be used as the waypoints for the spline and takes in a number (maximum is 3) to control the size of the curve between the waypoints and returns the path object.

* Path:CalculateUniformCFrame(T)
  
Takes in a T value from 0-1 and returns a uniform cframe across the path object.

* Path:CalculateUniformPosition(T)
  
Takes in a T value from 0-1 and returns a uniform position across the path object.

* Path:GetPathLength()

Returns the path objects length.
