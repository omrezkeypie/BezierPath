# ABOUT
BezierPath is an easy to use bezier spline module, designed to be used for TD games and general paths while being optimizied for large scale uses.

__NOTE__: BezierPath is NOT a traditional bezier spline. it behaves more like a linear spline that curves around each point.

__NOTE__: BezierPath has precomputation in it already. so no need to cache anything in your projects.

BezierPath can be installed using Wally:
```
BezierPath = "omrezkeypie/bezierpath@1.1.0"
```

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


## API

```lua
BezierPath.new(Positions : {Vector3},CurveSize : number) : Path
```
Takes in a table of positions which will be used as the waypoints for the spline and takes in a number to control the size of the curve between the waypoints and returns the path object.

```lua
Path:CalculateUniformCFrame(T : number) : CFrame
```

Takes in a T value from 0-1 and returns a uniform cframe across the path object.

```lua
Path:CalculateUniformPosition(T : number) : Position
```
  
Takes in a T value from 0-1 and returns a uniform position across the path object.

```lua
Path:GetPathLength() : number
```

Returns the path objects length.

```lua
Path:CalculateDerivative(T : number) : Vector3
```
  
Takes in a T value from 0-1 and returns the derivative of that T value.

```lua
Path:CalculateClosestPoint(Position : Vector3) : (Vector3,number)
```

Takes in a Vector3 position and returns the closest point and the closest T value on the path to the inputted position.
