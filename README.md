# ABOUT
BezierPath is an easy to use bezier spline module, designed to be used for TD games and general paths while being optimizied for large scale uses.

__NOTE__: BezierPath is NOT a traditional bezier spline. it behaves more like a linear spline that curves around each point.

BezierPath can be installed using Wally:
```
BezierPath = "omrezkeypie/bezierpath@2.0.3"
```

To view the API and usage, go [here](https://github.com/omrezkeypie/BezierPath/wiki/API)

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
Part.Size = Vector3.new(1,1,1)
Part.Anchored = true

for t = 0,1,1/100 do
    Part.CFrame = NewPath:CalculateUniformCFrame(t)
    task.wait(0.1)
end
```
A short video showing BezierPath.



https://github.com/omrezkeypie/BezierPath/assets/104690138/5e3fe18a-c8d3-4f0d-a475-55a3bfca6793
