--!native
--!strict

type PrecomputedCache = { any }

type LookUp = {
	Distances: { number }
}

type Section = {
	Positions: { Vector3 },
	Length: number,
	LookUp: LookUp | any
}

type self = {
	Sections: { Section },
	PathLength: number,
	PathLookup: { [Section]: { number } },
	TotalDistance: number,
	SampleAmount: number,
	PrecomputedCache: {
		Positions: { PrecomputedCache },
		CFrames: { PrecomputedCache },
	},
	CurveSize: number,
	ITERATION_AMOUNT: number
}

type Module = {
	__index: Module,
	new: (Waypoints: { Vector3 }, CurveSize: number) -> Path,
	GetPathLength: (self: Path) -> number,
	_Map: (self: Path, Value: number, In_min: number, In_max: number, Out_min: number, Out_max: number) -> number,
	_CalculateSectionPosition: (self: Path, Positions: { Vector3 }, t: number) -> Vector3,
	_CalculateDerivative: (self: Path, Positions: { Vector3 }, t: number) -> Vector3,
	_CalculateCFrame: (self: Path, Positions: { Vector3 }, t: number) -> CFrame,
	_CalculatePrecomputationCFrame: (self: Path, T: number) -> CFrame,
	CalculateUniformCFrame: (self: Path, T: number) -> CFrame,
	CalculateUniformPosition: (self: Path, T: number) -> Vector3,
	CalculateDerivative: (self: Path, T: number) -> Vector3,
	CalculateClosestPoint: (self: Path, Position: Vector3,Iterations : number?) -> Vector3,
	_InterpolateTPath: (self: Path, T: number) -> Section,
	_InterpolateT: (self: Path, Lookup: LookUp, T1: number) -> number,
	_PrecomputeUniformPositions: (self: Path) -> (),
	_CalculateLength: (self: Path, Positions: { Vector3 }) -> number,
	_CreateSectionLookup: (self: Path, Section: Section) -> LookUp,
	_ClampDistance: (self: Path, Position1: Vector3, Position2: Vector3) -> number,
	_Setup: (self: Path, StartingPositions: { Vector3 }) -> (),
	_CalculatePathLength: (self: Path) -> (),
	_CreatePathLookup: (self: Path) -> ()
}

local BezierPath: Module = {} :: Module
BezierPath.__index = BezierPath
local DEFAULT_EPSILON = 100
local DEFAULT_ITERATIONS = 20

local function Bezier(p0: Vector3, p1: Vector3, p2: Vector3, t: number): Vector3 
	return p1 + (1-t)^2*(p0 - p1)+t^2*(p2 - p1)
end

local function BezierDerivative(p0: Vector3, p1: Vector3, p2: Vector3, t: number): Vector3
	return 2*(1 - t)*(p1-p0) + 2*t*(p2-p1)
end

local function Lerp(p0: Vector3, p1: Vector3, t: number): Vector3
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
	newPath.ITERATION_AMOUNT = 0
	newPath:_Setup(Waypoints)

	return newPath
end

function BezierPath:GetPathLength(): number 
	return self.PathLength
end

function BezierPath:_Map(Value: number, In_min: number, In_max: number, Out_min: number, Out_max: number): number
	return (Value - In_max) * (Out_max - Out_min) / (In_max - In_min) + Out_min
end

function BezierPath:_CalculateSectionPosition(Positions: { Vector3 }, t: number): Vector3
	return Bezier(Positions[1], Positions[2], Positions[3], t)
end

function BezierPath:_CalculateCFrame(Positions: { Vector3 }, t: number): CFrame --More effiecent than using the derivative
	local pos = Bezier(Positions[1], Positions[2], Positions[3], t)
	local lookAt = Bezier(Positions[1], Positions[2], Positions[3], t + 1/DEFAULT_EPSILON)
	return CFrame.new(pos, lookAt)
end

--function PathBezier:CalculateCFrame(Positions,t)
--	local pos = self:_CalculatePosition(Positions,t)
--	local der = self:_CalculateDerivative(Positions,t)

--	return CFrame.new(pos, pos+der)
--end

function BezierPath:_CalculatePrecomputationCFrame(T: number): CFrame
	local PathSection = self:_InterpolateTPath(T)
	local InterpolatedT = self:_InterpolateT(PathSection.LookUp,T)

	return self:_CalculateCFrame(PathSection.Positions,InterpolatedT)
end

function BezierPath:CalculateUniformCFrame(T: number): CFrame
	local TranslatedIndex = math.min(math.floor(math.clamp(T,0,1) * self.ITERATION_AMOUNT),self.ITERATION_AMOUNT - 1)
	local FirstSample = self.PrecomputedCache["CFrames"][TranslatedIndex]
	local SecondSample = self.PrecomputedCache["CFrames"][math.min(TranslatedIndex + 1,self.ITERATION_AMOUNT - 1)]

	local SampleSubtraction = (SecondSample[2] - FirstSample[2])
	local Progress = (T - FirstSample[2]) / SampleSubtraction

	if SampleSubtraction == 0 then Progress = 1 end

	return FirstSample[1]:Lerp(SecondSample[1],Progress)
end

function BezierPath:CalculateUniformPosition(T: number): Vector3
	local TranslatedIndex = math.min(math.floor(math.clamp(T,0,1) * self.ITERATION_AMOUNT),self.ITERATION_AMOUNT - 1)
	local FirstSample = self.PrecomputedCache["Positions"][TranslatedIndex]
	local SecondSample = self.PrecomputedCache["Positions"][math.min(TranslatedIndex + 1,self.ITERATION_AMOUNT - 1)]

	local SampleSubtraction = (SecondSample[2] - FirstSample[2])
	local Progress = (T - FirstSample[2]) / SampleSubtraction

	if SampleSubtraction == 0 then Progress = 1 end

	return Lerp(FirstSample[1],SecondSample[1],Progress)
end

function BezierPath:CalculateDerivative(T : number) : Vector3
	T = math.clamp(T,0,1)
	
	local PathSection = self:_InterpolateTPath(T)
	local InterpolatedT = self:_InterpolateT(PathSection.LookUp,T)
	local Positions = PathSection.Positions
	
	return BezierDerivative(Positions[1],Positions[2],Positions[3],InterpolatedT)
end

function BezierPath:CalculateClosestPoint(Position : Vector3,Iterations : number?) : (Vector3 , number)
	local Start = 0
	local Middle = 0.5
	local End = 1
	local Iteration = 0
	local MaxIterations = Iterations or DEFAULT_ITERATIONS
	local ClosestT

	while Iteration < MaxIterations do
		local Position1,Position2,Position3 = self:CalculateUniformPosition(Start),self:CalculateUniformPosition(Middle),self:CalculateUniformPosition(End)
		local StartDistance = (Position1 - Position).Magnitude
		local MiddleDistance = (Position2 - Position).Magnitude
		local EndDistance = (Position3 - Position).Magnitude

		if StartDistance < EndDistance and StartDistance < MiddleDistance and MiddleDistance < EndDistance then
			ClosestT = Start

			End = Middle
			Middle = (Start + Middle) / 2
		elseif EndDistance < StartDistance and EndDistance < MiddleDistance and MiddleDistance < StartDistance then
			ClosestT = End

			Start =  Middle
			Middle = (Start + End) / 2
		elseif MiddleDistance < StartDistance and StartDistance < EndDistance then
			ClosestT = Middle

			End = Middle
			Middle = (Start + Middle) / 2
		else
			ClosestT = Middle

			Start = Middle
			Middle = (End + Middle) / 2
		end

		Iteration += 1
	end

	return self:CalculateUniformPosition(ClosestT),ClosestT
end

function BezierPath:_InterpolateTPath(T: number): Section
	for Section,Portion in self.PathLookup do
		if Portion[1] >= T and Portion[2] <= T then
			return Section
		end
	end

	return {} :: Section
end

function BezierPath:_InterpolateT(Lookup: LookUp, T1: number): number
	local distances = Lookup.Distances
	local n = #distances - 1 
	local targetDistance = self.PathLength * T1

	local lo = 0
	local hi = #distances

	repeat
		local i = lo + (hi - lo) // 2
		local value = distances[i + 1]
		local previousValue = distances[i]

		if previousValue <= targetDistance and value >= targetDistance then
			return self:_Map(
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

function BezierPath:_PrecomputeUniformPositions()
	local step = 1 / (self.ITERATION_AMOUNT - 1)

	for t = 0, 1, step do
		local index = math.floor(t * self.ITERATION_AMOUNT)
		local CalculatedCFrame = self:_CalculatePrecomputationCFrame(t)

		self.PrecomputedCache["CFrames"][index] = {CalculatedCFrame,t}
		self.PrecomputedCache["Positions"][index] = {CalculatedCFrame.Position,t}
	end

	local index = self.ITERATION_AMOUNT - 1
	local CalculatedCFrame = self:_CalculatePrecomputationCFrame(1)

	self.PrecomputedCache["CFrames"][index] = {CalculatedCFrame,1}
	self.PrecomputedCache["Positions"][index] = {CalculatedCFrame.Position,1}
end

function BezierPath:_CalculateLength(Positions: { Vector3 }): number
	local Length = 0
	local Epsilon = 1/DEFAULT_EPSILON

	for i = 0,1,Epsilon do
		local Pos1,Pos2 = self:_CalculateSectionPosition(Positions,i),self:_CalculateSectionPosition(Positions,i+Epsilon)
		Length += (Pos1 - Pos2).Magnitude
	end

	return Length
end

function BezierPath:_CreateSectionLookup(Section: Section): LookUp
	local LookUp = {
		Distances = {}
	}
	local Segments =  math.floor(30 * (Section.Length + 0.5))

	local prevPosition = self:_CalculateSectionPosition(Section.Positions,0)
	local AccumulatedDistance = self.TotalDistance

	for i = 0, Segments do
		local Position = self:_CalculateSectionPosition(Section.Positions, i / Segments)
		local deltaPosition = prevPosition - Position
		local SegmentLength = deltaPosition.Magnitude
		AccumulatedDistance = AccumulatedDistance + SegmentLength
		LookUp.Distances[i] = AccumulatedDistance
		prevPosition = Position
	end

	self.TotalDistance = AccumulatedDistance
	LookUp.TotalDistance = AccumulatedDistance

	return LookUp
end

function BezierPath:_ClampDistance(Position1: Vector3, Position2: Vector3): number
	local Distance = (Position1 - Position2).Magnitude

	if Distance < self.CurveSize * 2 then return Distance / 2 end

	return self.CurveSize
end

function BezierPath:_Setup(StartingPositions: { Vector3 })
	local newWaypoints = {}

	table.insert(newWaypoints,StartingPositions[1])

	table.insert(newWaypoints,StartingPositions[1] - (StartingPositions[1] - StartingPositions[2]).Unit * self:_ClampDistance(StartingPositions[1],StartingPositions[2]))

	for i = 2,#StartingPositions-1 do
		local CurrentPosition = StartingPositions[i]
		local NextPosition = StartingPositions[i + 1]
		local PreviousPosition = StartingPositions[i - 1]
		local Positions = {
			CurrentPosition - ((CurrentPosition - PreviousPosition).Unit * self:_ClampDistance(CurrentPosition,PreviousPosition)),
			CurrentPosition - ((CurrentPosition - NextPosition).Unit * self:_ClampDistance(CurrentPosition,NextPosition)),
			CurrentPosition - ((CurrentPosition - NextPosition).Unit * self:_ClampDistance(CurrentPosition,NextPosition))
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
			Length = self:_CalculateLength(Positions),
			LookUp = nil,
		}
		table.insert(self.Sections,Section)
	end

	self:_CalculatePathLength()

	for _,Section in self.Sections do
		Section.LookUp = self:_CreateSectionLookup(Section)
	end

	self:_CreatePathLookup()

	self.ITERATION_AMOUNT = math.floor(self:GetPathLength() * 8)

	self:_PrecomputeUniformPositions()
end

function BezierPath:_CalculatePathLength()
	for i,section in self.Sections do
		self.PathLength += section.Length
	end
end

function BezierPath:_CreatePathLookup()
	local AccumalatedLength = 0
	for i,Section in self.Sections do
		AccumalatedLength += Section.Length
		local PortionOfPath = AccumalatedLength / self.PathLength
		self.PathLookup[Section] = {PortionOfPath,(AccumalatedLength - Section.Length) / self.PathLength}
	end
end

export type Path = typeof(setmetatable({} :: self, {} :: Module))

return BezierPath
