type LookAt = {
	Distances: { number }
}

type Section = {
	Positions: { Vector3 },
	Length: number,
	LookAt: LookAt?
}

type self = {
	Sections: { Section },
	PathLenght: number,
	PathLookUp: { [Section]: { number } },
	TotalDistance: number,
	SampleAmount: number,
	PrecomputedCache: {} :: {
		Positions: { Vector3 },
		CFrames: { CFrame }
	},
	CurveSize: number
}

type Module = {
	__index: Module,
	new: (Waypoints: { Vector3 }) -> Path,
	GetPartLength: (self: Path) -> number,
	Map: (self: Path, Value: number, In_min: number, In_max: number, Out_min: number, Out_max: number) -> number,
	CalculateSectionPosition: (self: Path, Positions: { Vector3 }, t: number) -> Vector3,
	CalculateDerivative: (self: Path, Positions: { Vector3 }, t: number) -> Vector3,
	CalculateCFrame: (self: Path, Positions: { Vector3 }, t: number) -> CFrame,
	CalculatePrecomputationCFrame: (self: Path, T: number) -> CFrame,
	CalculatePrecomputationPosition: (self: Path, T: number) -> Vector3,
	CalculateUniformCFrame: (self: Path, T: number) -> CFrame,
	CalculateUniformPosition: (self: Path, T: number) -> Vector3,
	InterpolateTPath: (self: Path, T: number) -> Section
	InterpolateT: (self: Path, Lookup: LookUp, T1: number) -> number,
	PrecomputeUniformPositions: (self: Path) -> (),
	CalculateLength: (self: Path, Positions: { Vector3 }) -> number,
	CreateSectionLookup: (self: Path, Section: Section) -> LookUp
	ClampDistance: (self: Path, Position1: Vector3, Position2: Vector3) -> number.
	Setup: (self: Path, StartingPositions: { Vector3 }) -> (),
	CalculatePathLength: (self: Path) -> (),
	CreatePathLookup: (self: Path) -> ()
}

local BezierPath = {} :: Module
BezierPath.__index = BezierPath
local DEFAULT_EPSILON = 100
local ITERATION_AMONT = 0

local function bez(p0,p1,p2,t)
	return p1 + (1-t)^2*(p0 - p1)+t^2*(p2 - p1)
end

local function bezder(p0,p1,p2,t)
	return 2*(1 - t)*(p1-p0) + 2*t*(p2-p1)
end

local function lerp(p0,p1,t)
	return p0 + t*(p1 - p0)
end


function BezierPath.new(Waypoints: { Vector3 }, CurveSize: number): Path
	local newPath = {}
	setmetatable(newPath,BezierPath)
	newPath.Sections = {}
	newPath.PathLength = 0
	newPath.PathLookup = {}
	newPath.TotalDistance = 0
	newPath.SampleAmount = 0
	newPath.PrecomputedCache = {
		Positions = {},
		CFrames = {}
	}
	newPath.CurveSize = CurveSize
	newPath:Setup(Waypoints)

	return newPath
end

function BezierPath:GetPathLength(): number 
	return self.PathLength
end

function BezierPath:Map(Value: number, In_min: number, In_max: number, Out_min: number, Out_max: number): number
	return (Value - In_max) * (Out_max - Out_min) / (In_max - In_min) + Out_min
end

function BezierPath:CalculateSectionPosition(Positions: { Vector3 }, t: number): Vector3
	return bez(Positions[1], Positions[2], Positions[3], t)
end

function BezierPath:CalculateDerivative(Positions: { Vector3 }, t: number): Vector3
	return bezder(Positions[1], Positions[2], Positions[3], t)
end

function BezierPath:CalculateCFrame(Positions: { Vector3 }, t: number): CFrame --More effiecent than using the derivative
	local pos = bez(Positions[1], Positions[2], Positions[3], t)
	local lookAt = bez(Positions[1], Positions[2], Positions[3], t + 1/DEFAULT_EPSILON)
	return CFrame.new(pos, lookAt)
end

--function PathBezier:CalculateCFrame(Positions,t)
--	local pos = self:CalculatePosition(Positions,t)
--	local der = self:CalculateDerivative(Positions,t)

--	return CFrame.new(pos, pos+der)
--end

function BezierPath:CalculatePrecomputationCFrame(T: number): CFrame
	local PathSection = self:InterpolateTPath(T)
	local InterpolatedT = self:InterpolateT(PathSection.LookUp,T)

	return self:CalculateCFrame(PathSection.Positions,InterpolatedT)
end

function BezierPath:CalculatePrecomputationPosition(T: number): Vector3
	local PathSection = self:InterpolateTPath(T)
	local InterpolatedT = self:InterpolateT(PathSection.LookUp,T)

	return self:CalculateSectionPosition(PathSection.Positions,InterpolatedT)
end

function BezierPath:CalculateUniformCFrame(T: number): CFrame
	local TranslatedIndex = math.floor(math.clamp(T,0,1) * (ITERATION_AMONT - 1) + 0.5)
	local FirstSample = self.PrecomputedCache["CFrames"][TranslatedIndex]
	local SecondSample = self.PrecomputedCache["CFrames"][math.clamp(TranslatedIndex + 1,0,ITERATION_AMONT - 1)]
	local Progress = (T - FirstSample[2]) / (SecondSample[2] - FirstSample[2])

	return FirstSample[1]:Lerp(SecondSample[1],Progress)
end

function BezierPath:CalculateUniformPosition(T: number): Vector3
	local TranslatedIndex = math.floor(math.clamp(T,0,1) * (ITERATION_AMONT - 1) + 0.5)
	local FirstSample = self.PrecomputedCache["Positions"][TranslatedIndex]
	local SecondSample = self.PrecomputedCache["Positions"][math.clamp(TranslatedIndex + 1,0,ITERATION_AMONT - 1)]
	local Progress = (T - FirstSample[2]) / (SecondSample[2] - FirstSample[2])

	return lerp(FirstSample[1],SecondSample[1],Progress)
end

function BezierPath:InterpolateTPath(T: number): Section
	for Section,Portion in self.PathLookup do
		if Portion[1] >= T and Portion[2] <= T then
			return Section
		end
	end

	return self.PathLookup[#self.PathLookup]
end

function BezierPath:InterpolateT(Lookup: LookUp, T1: number): number
	local distances = Lookup.Distances
	local n = #distances - 1 
	local targetDistance = self.PathLength * T1

	local lo = 0
	local hi = #distances

	repeat
		local i = math.floor(lo + (hi - lo) / 2)
		local value = distances[i + 1]
		local previousValue = distances[i]

		if previousValue <= targetDistance and value >= targetDistance then
			return self:Map(
				targetDistance,

				previousValue,
				value,

				i / n,
				(i + 1) / n
			)  
		elseif value > targetDistance then
			hi = i
		else
			lo = i + 1
		end
	until lo >= hi

	return 1
end

function BezierPath:PrecomputeUniformPositions()
	local step = 1 / (ITERATION_AMONT - 1)

	for t = 0, 1, step do
		local index = math.floor(t * (ITERATION_AMONT - 1) + 0.5)
		local CalculatedCFrame = self:CalculatePrecomputationCFrame(t)

		self.PrecomputedCache["CFrames"][index] = {CalculatedCFrame,t}
		self.PrecomputedCache["Positions"][index] = {CalculatedCFrame.Position,t}
	end
end

function BezierPath:CalculateLength(Positions: { Vector3 }): number
	local Length = 0
	local Epsilon = 1/DEFAULT_EPSILON

	for i = 0,1,Epsilon do
		local Pos1,Pos2 = self:CalculateSectionPosition(Positions,i),self:CalculateSectionPosition(Positions,i+Epsilon)
		Length += (Pos1 - Pos2).Magnitude
	end

	return Length
end

function BezierPath:CreateSectionLookup(Section: Section): LookUp
	local LookUp = {
		Distances = {}
	}
	local Segments =  math.floor(30 * Section.Length)
	local prevPosition = self:CalculateSectionPosition(Section.Positions,0)
	local AccumulatedDistance = self.TotalDistance

	for i = 0,Segments do
		local Position = self:CalculateSectionPosition(Section.Positions,i / Segments)
		local SegmentLength = (prevPosition - Position).Magnitude
		AccumulatedDistance += SegmentLength
		LookUp.Distances[i] = AccumulatedDistance
		prevPosition = Position
	end

	self.TotalDistance = AccumulatedDistance
	LookUp.TotalDistance = AccumulatedDistance

	return LookUp
end

function BezierPath:ClampDistance(Position1: Vector3, Position2: Vector3): number
	local Distance = (Position1 - Position2).Magnitude

	if Distance < self.CurveSize then return Distance / self.CurveSize end

	return self.CurveSize
end

function BezierPath:Setup(StartingPositions: { Vector3 })
	local newWaypoints = {}

	table.insert(newWaypoints,StartingPositions[1])

	table.insert(newWaypoints,StartingPositions[1] - (StartingPositions[1] - StartingPositions[2]).Unit * self:ClampDistance(StartingPositions[1],StartingPositions[2]))

	for i = 2,#StartingPositions-1 do
		local CurrentPosition = StartingPositions[i]
		local NextPosition = StartingPositions[i + 1]
		local PreviousPosition = StartingPositions[i - 1]
		local Positions = {
			CurrentPosition - (CurrentPosition - PreviousPosition).Unit * self:ClampDistance(CurrentPosition,PreviousPosition),
			CurrentPosition - (CurrentPosition - NextPosition).Unit * self:ClampDistance(CurrentPosition,NextPosition),
			CurrentPosition - (CurrentPosition - NextPosition).Unit * self:ClampDistance(CurrentPosition,NextPosition) * 1.1
		}  

		for j = 1,3 do
			table.insert(newWaypoints,Positions[j])
			if j == 1 then
				table.insert(newWaypoints,CurrentPosition)
			end
		end 
	end


	table.insert(newWaypoints,StartingPositions[#StartingPositions])


	for Index = 2,#newWaypoints,2 do
		local Positions = {
			newWaypoints[Index - 1],
			newWaypoints[Index],
			newWaypoints[Index + 1],
		}
		local Section = {
			Positions = Positions,
			Length = self:CalculateLength(Positions),
			LookUp = nil,
		}
		table.insert(self.Sections,Section)
	end

	self:CalculatePathLength()

	for _,Section in pairs(self.Sections) do
		Section.LookUp = self:CreateSectionLookup(Section)
	end

	self:CreatePathLookup()

	ITERATION_AMONT = math.floor(self:GetPathLength() * 8)

	self:PrecomputeUniformPositions()
end

function BezierPath:CalculatePathLength()
	for i,section in pairs(self.Sections) do
		self.PathLength += section.Length
	end
end

function BezierPath:CreatePathLookup()
	local AccumalatedLength = 0
	for i,Section in pairs(self.Sections) do
		AccumalatedLength += Section.Length
		local PortionOfPath = AccumalatedLength / self.PathLength
		self.PathLookup[Section] = {PortionOfPath,(AccumalatedLength - Section.Length) / self.PathLength}
	end
end

export type Path = typeof(setmetatable({} :: self, {} :: Module))

return BezierPath  
