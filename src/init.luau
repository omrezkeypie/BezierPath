--!optimize 2
--!native
--!strict

local EPSILON = 100
local DEFAULT_CURVE_SIZE = 3

type Section = {
	Positions : {Vector3},
	Length : number,
	Type : number,
	TRanges : {number},
	AccumulatedDistance : number,
	DistanceLookup : {number}
}

type Methods = {
	CalculateUniformPosition: (self : Path,T : number) -> Vector3,
	CalculateUniformCFrame: (self : Path,T : number) -> CFrame,
	CalculateDerivative: (self : Path,T : number) -> Vector3,
	GetPathLength: (self : Path) -> number,
	CalculateClosestPoint: (self : Path,Position : Vector3,Iterations : number?) -> (CFrame,number),
	VisualizePath: (self : Path) -> (),
}

export type Path = {
	Sections : {Section},
	PathLength : number,
} & Methods

type Module = {
	new: (Waypoints : {Vector3},CurveSize : number) -> Path,
}

local BezierPath : Module = {} :: Module
local Methods : Methods = {} :: Methods

local function Map(Value: number, In_min: number, In_max: number, Out_min: number, Out_max: number): number
	return (Value - In_min) * (Out_max - Out_min) / (In_max - In_min) + Out_min
end

local function Bezier(p0: Vector3, p1: Vector3, p2: Vector3, t: number) : Vector3 
	return p1 + (1-t)^2*(p0 - p1)+t^2*(p2 - p1)
end

local function BezierDerivative(p0: Vector3, p1: Vector3, p2: Vector3, t: number) : Vector3
	return 2*(1 - t)*(p1-p0) + 2*t*(p2-p1)
end

local function Lerp(p0: Vector3, p1: Vector3, t: number) : Vector3
	return p0 + t*(p1 - p0)
end

local function CalculateSectionPosition(Section : Section,T : number) : Vector3
	local Positions = Section.Positions

	if Section.Type == 1 then
		return Lerp(Positions[1],Positions[2],T)
	else
		return Bezier(Positions[1],Positions[2],Positions[3],T)
	end
end

local function CalculateSectionDerivative(Section : Section,T : number) : Vector3
	local Positions = Section.Positions

	return BezierDerivative(Positions[1],Positions[2],Positions[3],T)
end

local function CalculateSectionCFrame(Section : Section,T : number) : CFrame
	local Position = CalculateSectionPosition(Section,T)
	local Derivative = CalculateSectionDerivative(Section,T)

	return CFrame.new(Position,Position + Derivative)
end

local function ClampCurveSize(Position1 : Vector3,Position2 : Vector3,CurveSize : number) : number
	local Distance = (Position1 - Position2).Magnitude

	if Distance < CurveSize * 2 then return Distance / 2 end

	return CurveSize
end

local function GetCurveSize(CurveSize : number | {number},Index : number) : number
	if typeof(CurveSize) == "number" then return CurveSize end

	return CurveSize[Index] or DEFAULT_CURVE_SIZE
end

local function SetupWaypoints(InputtedWaypoints : {Vector3},CurveSize : number | {number}) : {Vector3}
	local Waypoints = {}

	table.insert(Waypoints,InputtedWaypoints[1]) 

	for i = 2,#InputtedWaypoints - 1 do
		local GottenCurveSize = GetCurveSize(CurveSize,i - 1)
		local Position = InputtedWaypoints[i]
		local InputtedPreviousPosition = InputtedWaypoints[i - 1]
		local InputtedNextPosition = InputtedWaypoints[i + 1]

		local PreviousPosition = Position - (Position - InputtedPreviousPosition).Unit * ClampCurveSize(Position,InputtedPreviousPosition,GottenCurveSize)
		local NextPosition =  Position - (Position - InputtedNextPosition).Unit * ClampCurveSize(Position,InputtedNextPosition,GottenCurveSize)

		table.insert(Waypoints,PreviousPosition)
		table.insert(Waypoints,Position)
		table.insert(Waypoints,NextPosition)
	end

	table.insert(Waypoints,InputtedWaypoints[#InputtedWaypoints])

	return Waypoints
end

local function CalculateSectionLength(Section : Section) : number
	local Length = 0

	for T = 0,1,1/EPSILON do
		local Position1 = CalculateSectionPosition(Section,T)
		local Position2 = CalculateSectionPosition(Section,T + 1/EPSILON)

		Length += (Position1 - Position2).Magnitude
	end

	return Length
end

local function CalculatePathLength(Sections : {Section}) : number
	local TotalLength = 0

	for _,Section in Sections do
		TotalLength += Section.Length
	end

	return TotalLength
end

local function CreateSectionsDistanceLookup(Sections : {Section})
	for _,Section in Sections do
		local AccumulatedDistance = Section.AccumulatedDistance
		local PreviousPosition = CalculateSectionPosition(Section,0)
		local Segments = math.floor(Section.Length * 100)

		for i = 1,Segments do
			local Position = CalculateSectionPosition(Section,i / Segments)
			local Distance = (Position - PreviousPosition).Magnitude

			Section.DistanceLookup[i] = AccumulatedDistance
			
			AccumulatedDistance += Distance
			
			PreviousPosition = Position
		end
	end
end

local function CreateSections(BezierWaypoints : {Vector3}) : {Section}
	local Sections = {}

	local Index = 2
	local Step = 1

	while Index <= #BezierWaypoints do
		local SectionPositions = {}

		local PreviousPosition = BezierWaypoints[Index - 1]
		local Position = BezierWaypoints[Index]
		local NextPosition = BezierWaypoints[Index + 1]

		if Step == 1 then
			table.insert(SectionPositions,PreviousPosition)
			table.insert(SectionPositions,Position)
			table.insert(SectionPositions,Position)
		else
			table.insert(SectionPositions,PreviousPosition)
			table.insert(SectionPositions,Position)
			table.insert(SectionPositions,NextPosition)
		end

		local newSection = {
			AccumulatedDistance = 0,
			Positions = SectionPositions,
			Type = Step,
			Length = 0,
			TRanges = {},
			DistanceLookup = {}
		}

		newSection.Length = CalculateSectionLength(newSection)

		Index += Step
		Step = Step == 1 and 2 or 1

		table.insert(Sections,newSection)
	end

	return Sections
end

local function SetupSectionsT(PathLength : number,Sections : {Section})
	local AccumulatedT = 0

	for i = 1,#Sections do
		local Section = Sections[i]
		local PortionOfPath = Section.Length / PathLength
		Section.TRanges = {AccumulatedT,AccumulatedT + PortionOfPath}

		AccumulatedT += PortionOfPath
	end
end

local function SetupSectionsAccumulatedDistance(Sections : {Section})
	local AccumulatedDistance = 0

	for SectionIndex = 1,#Sections do
		local Section = Sections[SectionIndex]		

		Section.AccumulatedDistance = AccumulatedDistance
		AccumulatedDistance += Section.Length
	end
end

local function MapT(Section : Section, PathLength : number, T : number) : number
	if T >= 1 then return 1 end
	if Section.Type == 1 then -- if it's a straight section, no need to map the T value, just linear interpolate
		return ((T * PathLength) - Section.AccumulatedDistance) / Section.Length
	end

	local InputtedDistance = T * PathLength
	local Low = 1
	local High = #Section.DistanceLookup - 1

	while Low <= High do 
		local i = math.floor((Low + High) / 2)
		local Distance = Section.DistanceLookup[i]
		local NextDistance = Section.DistanceLookup[i + 1]

		if InputtedDistance >= Distance and InputtedDistance <= NextDistance then
			return Map(
				InputtedDistance,
				Distance,
				NextDistance,
				i / (#Section.DistanceLookup - 1),
				(i + 1) / (#Section.DistanceLookup - 1)
			)
		elseif InputtedDistance < Distance then
			High = i - 1
		else
			Low = i + 1
		end
	end

	return 1
end

local function GetSectionFromT(Sections : {Section},T : number) : Section
	for _,Section in Sections do
		if Section.TRanges[1] <= T and Section.TRanges[2] > T then
			return Section
		end
	end

	return Sections[#Sections]
end

local function LoadMethods(Object : {})
	for FunctionName,Function in Methods do
		Object[FunctionName] = Function
	end
end

function BezierPath.new(Waypoints : {Vector3},CurveSize : number | {number}) : Path
	local BezierWaypoints = SetupWaypoints(Waypoints,CurveSize)

	local Sections = CreateSections(BezierWaypoints)
	local PathLength = CalculatePathLength(Sections)
	SetupSectionsT(PathLength,Sections)
	SetupSectionsAccumulatedDistance(Sections)
	CreateSectionsDistanceLookup(Sections)

	local newPath : Path = {
		Sections = Sections,
		PathLength = PathLength
	} :: Path

	LoadMethods(newPath)

	return newPath
end

function Methods:GetPathLength() : number
	return self.PathLength
end

function Methods:CalculateUniformPosition(T : number) : Vector3
	local Section = GetSectionFromT(self.Sections,T)
	local MappedT = MapT(Section,self.PathLength,T)

	return CalculateSectionPosition(Section,MappedT)
end

function Methods:CalculateUniformCFrame(T : number) : CFrame
	local Section = GetSectionFromT(self.Sections,T)
	local MappedT = MapT(Section,self.PathLength,T)

	return CalculateSectionCFrame(Section,MappedT)
end

function Methods:CalculateDerivative(T : number) : Vector3
	local Section = GetSectionFromT(self.Sections,T)
	local MappedT = MapT(Section,self.PathLength,T)

	return CalculateSectionDerivative(Section,MappedT)
end

function Methods:CalculateClosestPoint(Position: Vector3) : (CFrame, number)
	local ClosestT = 0
	local ClosestDistance = math.huge

	for T = 0,1,1/100 do
		local PathPosition = self:CalculateUniformPosition(T)
		local Distance = (PathPosition - Position).Magnitude

		if Distance < ClosestDistance then
			ClosestDistance = Distance
			ClosestT = T
		end
	end

	return self:CalculateUniformCFrame(ClosestT), ClosestT
end 

local function MakePart(InputtedCFrame : CFrame)
	local Part = Instance.new("Part")
	Part.Anchored = true
	Part.Transparency = 0.5
	Part.CanCollide = false
	Part.Size = Vector3.one
	Part.Color = Color3.new(1, 0.235294, 0.247059)
	Part.Material = Enum.Material.SmoothPlastic
	Part.CFrame = InputtedCFrame
	Part.Parent = workspace
end

function Methods:VisualizePath()
	for T = 0,1,1/100 do
		MakePart(self:CalculateUniformCFrame(T))
	end
end

return BezierPath 
