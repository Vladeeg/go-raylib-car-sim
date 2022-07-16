package main

import (
	"fmt"
	"math"

	"github.com/gen2brain/raylib-go/raylib"
)

// Flongtitude = Ftraction + Fdrag + FrollingRes

// Ftraction = heading * Engineforce

// FrollingRes = -Crr * v

// Tdrive =  Tengine * xg * xd * n

// Fmax = mu * W
// where mu is resistance coeff

// Weight transfer
// Wf = (c/L)*W - (h/L)*M*a
// Wr = (b/L)*W + (h/L)*M*a
// where W = Mg
// c, b - distances from center of gravity to axles
// L - wheelbase

// Fdrag =  0.5 * frictionCoeff * frontalArea * airDensity * velocity ^ 2
// rho = 1.29 kg/m3 // could be lower while slipstreaming?
// Cdrag = 0.5 * frictionCoeff * frontalArea * 1.29
// CrollingRes ~= 30 * Cdrag

// Fdrive = u * Tengine * xg * xd * n / Rw
// where
// u is a unit vector which reflects the car's orientation,
// Tengine is the torque of the engine at a given rpm,
// xg is the gear ratio,
// xd is the differential ratio,
// n is transmission efficiency and
// Rw is wheel radius.

/*
Mass               M  1439 kg
Wheel radius          0.34 m

First gear              g1 2.66
Second gear             g2 1.78
Third gear              g3 1.30
Fourth gear             g4 1.0
Fifth gear              g5 0.74
Sixth (!) gear          g6 0.50
Reverse                 gR 2.90
Differential ratio      xd 3.42
Transmission efficiency    0.7
*/

type Transmission struct {
	Gears      []float32
	DiffRatio  float32
	Efficiency float32
	Gear       int
}

type Engine struct {
	TorqueCurve     [][]float32 // RPM to N*m mapping
	FlywheelInertia float32
	MinRpm          float32
	Rpm             float32
}

type Car struct {
	Direction          rl.Vector2 // Unit vector rotated by Rotation
	Rotation           float32
	Mass               float32 // In kilograms
	WheelRadius        float32 // In meters
	FrictionCoeff      float32
	FrontalArea        float32
	Transmission       Transmission
	Engine             Engine
	Position           rl.Vector2
	SteeringAngle      float32
	MaxSteeringAngle   float32
	AngularVelocity    float32
	Velocity           rl.Vector2
	Acceleration       rl.Vector2
	Throttle           float32
	Brake              float32
	WheelMu            float32
	Wheelbase          float32
	WeightDistribution float32 // Weight on front
	Dimensions         rl.Vector2
	CGHeight           float32
}

var (
	pixelsInMeter = 10
	trail         = [][]rl.Vector2{}
	// Toyota GT86
	car = Car{
		SteeringAngle:    0,
		MaxSteeringAngle: 35 * math.Pi / 180,
		Direction:        rl.Vector2{X: 0, Y: -1},
		Rotation:         0,
		AngularVelocity:  0,
		Mass:             1239,
		WheelRadius:      0.33,
		FrictionCoeff:    0.3,
		FrontalArea:      2.0,
		Transmission: Transmission{
			Gears: []float32{
				-3.437, // Reverse
				0,      // Neutral
				3.626,
				2.188,
				1.541,
				1.213,
				1,
				0.767,
			},
			DiffRatio:  4.1,
			Efficiency: 0.9,
			Gear:       1,
		},
		Engine: Engine{
			MinRpm: 800,
			Rpm:    800,
			TorqueCurve: [][]float32{
				{0, 0},
				{800, 100},
				{1000, 140},
				{3000, 205},
				{6000, 210},
				{7000, 180},
			},
			FlywheelInertia: 5 * (0.20 * 0.20),
		},
		Position:           rl.Vector2{X: 10, Y: 10},
		Wheelbase:          2.57,
		WeightDistribution: 0.55,
		Dimensions: rl.Vector2{
			X: 1.775,
			Y: 4.24,
		},
		Velocity:     rl.Vector2{},
		WheelMu:      1,
		Acceleration: rl.Vector2{},
		CGHeight: 0.457,
	}

	airDensity float32 = 1.29

	velocityWc rl.Vector2 = rl.Vector2{}
	// accelerationWc rl.Vector2 = rl.Vector2{}

	// slipAngleCurve = [][]float32{
	// 	{-20 * math.Pi / 180, 4200.0 / 5000.0},
	// 	{-3 * math.Pi / 180, 6000.0 / 5000.0},
	// 	{0, 0},
	// 	{3 * math.Pi / 180, -6000.0 / 5000.0},
	// 	{20 * math.Pi / 180, -4200.0 / 5000.0},
	// }

	rect rl.Rectangle = rl.Rectangle{
		X: car.Position.X, Y: car.Position.Y,
		Width:  car.Dimensions.X,
		Height: car.Dimensions.Y,
	}

	wheelRect rl.Rectangle = rl.Rectangle{
		X: car.Position.X, Y: car.Position.Y,
		Width:  0.215,
		Height: car.WheelRadius * 2,
	}
)

func LookupAndInterpolate(table [][]float32, point float32) float32 {
	points := len(table)

	for i := 0; i < points-1; i++ {
		min := table[i]
		max := table[i+1]
		if point >= min[0] && point <= max[0] {
			diffx := point - min[0]
			diffn := max[0] - min[0]

			return min[1] + (max[1]-min[1])*diffx/diffn
		}
	}

	return 0
}

func (e Engine) FlywheelAccel(torque float32) float32 {
	return torque / e.FlywheelInertia
}

func RpmToAngularVelocity(rpm float32) float32 {
	return rpm * 2 * math.Pi
}

func AngularVelocityToRpm(angularVelocity float32) float32 {
	return angularVelocity / (2 * math.Pi)
}

func EngineRpmFromWheelRpm(
	wheelRotationRate float32,
	gearRatio float32,
	diffRatio float32,
) float32 {
	return wheelRotationRate * gearRatio * car.Transmission.DiffRatio * 60 / (2 * math.Pi)
}

func WheelSpeedmFromEngineRpm(
	rpm float32,
	gearRatio float32,
	diffRatio float32,
	transmissionEfficiency float32,
	wheelRadius float32,
) float32 {
	return (rpm / (gearRatio * car.Transmission.DiffRatio * 60)) * (2 * math.Pi) * wheelRadius
}

func TractionForce(
	eTorque float32,
	gearRatio float32,
	diffRatio float32,
	transmissionEfficiency float32,
	wheelRadius float32,
) float32 {
	return eTorque * gearRatio * car.Transmission.DiffRatio * transmissionEfficiency / wheelRadius
}

func Drag(
	frictionCoeff float32,
	frontalArea float32,
	airDensity float32,
) float32 {
	return 0.5 * frictionCoeff * frontalArea * airDensity
}

func Signum(v float32) int {
	if v < 0 {
		return -1
	} else if v > 0 {
		return 1
	} else {
		return 0
	}
}

func Clamp(v int, min int, max int) int {
	if v < min {
		return min
	}
	if v > max {
		return max
	}
	return v
}

func FloatClamp(v float32, min float32, max float32) float32 {
	if v < min {
		return min
	}
	if v > max {
		return max
	}
	return v
}

func WheelRotationRateFromVelocity(velocity float32, wheelRadius float32) float32 {
	return velocity / wheelRadius
}

func AddToTrail(v []rl.Vector2) {
	if len(trail) < 200 {
		trail = append(trail, v)
	} else {
		trail = trail[1:]
		trail = append(trail, v)
	}
}

type Control struct {
	Throttle float32
	Brake    float32
	Steering float32
}

func Max(l float32, r float32) float32 {
	if l < r {
		return r
	}
	return l
}

func Min(l float32, r float32) float32 {
	if l < r {
		return l
	}
	return r
}

func Controls() Control {
	throttleAnalog := (1 + rl.GetGamepadAxisMovement(0, 5)) / 2
	brakeAnalog := (1 + rl.GetGamepadAxisMovement(0, rl.GamepadXboxAxisLt)) / 2

	var throttleDiscrete float32 = 0
	if rl.IsKeyDown(rl.KeyUp) {
		throttleDiscrete = 1
	}
	var brakeDiscrete float32 = 0
	if rl.IsKeyDown(rl.KeyDown) {
		brakeDiscrete = 1
	}

	steeringAnalog := rl.GetGamepadAxisMovement(0, rl.GamepadXboxAxisLeftX)
	steeringAnalog = -float32(Signum(steeringAnalog)) * float32(math.Pow(math.Abs(float64(steeringAnalog)), 1.75))

	// rl.DrawText(fmt.Sprintf("%f", steeringAnalog), 10, 400, 20, rl.DarkGray)

	var steeringDiscrete float32 = 0
	if rl.IsKeyDown(rl.KeyLeft) {
		steeringDiscrete = 1
	}
	if rl.IsKeyDown(rl.KeyRight) {
		steeringDiscrete = -1
	}

	var steering float32 = steeringDiscrete

	if math.Abs(float64(steeringAnalog)) > math.Abs(float64(steeringDiscrete)) {
		steering = steeringAnalog
	}

	return Control{
		Throttle: Max(throttleAnalog, throttleDiscrete),
		Brake:    Max(brakeAnalog, brakeDiscrete),
		Steering: car.MaxSteeringAngle * steering,
	}
}

func Run(delta float32) {
	controls := Controls()
	car.Throttle = controls.Throttle
	car.Brake = controls.Brake
	car.SteeringAngle = controls.Steering

	car.SteeringAngle = rl.Clamp(car.SteeringAngle, -car.MaxSteeringAngle, car.MaxSteeringAngle)

	if rl.IsKeyPressed(rl.KeyLeftShift) {
		car.Transmission.Gear += 1
	} else if rl.IsKeyPressed(rl.KeyLeftControl) {
		car.Transmission.Gear -= 1
	}

	if rl.IsGamepadButtonPressed(0, 7) {
		car.Transmission.Gear += 1
	} else if rl.IsGamepadButtonPressed(0, 8) {
		car.Transmission.Gear -= 1
	}

	car.Transmission.Gear = Clamp(
		car.Transmission.Gear,
		0,
		len(car.Transmission.Gears)-1,
	)

	cs := float32(math.Cos(float64(car.Rotation)))
	sn := float32(math.Sin(float64(car.Rotation)))

	// yawspeed := car.Wheelbase * 0.5 * car.AngularVelocity
	// var rot_angle float32 = 0
	// if math.Abs(float64(car.Velocity.X)) > 0.1 {
	// 	rot_angle = float32(math.Atan2(float64(yawspeed), float64(car.Velocity.X)))
	// }
	// var sideslip float32 = 0
	// if math.Abs(float64(car.Velocity.X)) > 0.1 {
	// 	sideslip = float32(math.Atan2(float64(car.Velocity.Y), float64(car.Velocity.X)))
	// }
	// slipFront := sideslip + rot_angle - car.SteeringAngle * float32(Signum(car.Velocity.X))
	// slipRear := sideslip - rot_angle

	var slipFront float32 = 0
	var slipRear float32 = 0
	
	if math.Abs(float64(car.Velocity.X)) > 0.1 {
		slipFront = float32(math.Atan(
			float64(car.Velocity.Y + car.AngularVelocity * car.WeightDistribution)/
			math.Abs(float64(car.Velocity.X)),
		)) - car.SteeringAngle * float32(Signum(car.Velocity.X))

		slipRear = float32(math.Atan(
			float64(car.Velocity.Y - car.AngularVelocity * (1-car.WeightDistribution))/
			math.Abs(float64(car.Velocity.X)),
		))
	}

	weightF := car.Mass*9.8*car.WeightDistribution - (car.CGHeight/car.Wheelbase)*car.Mass*car.Acceleration.X
	weightR := car.Mass*9.8*(1-car.WeightDistribution) + (car.CGHeight/car.Wheelbase)*car.Mass*car.Acceleration.X

	lateralForceFront := rl.Vector2{
		X: 0,
		Y: FloatClamp(-5*slipFront, -2, 2) * weightF,
		// Y: LookupAndInterpolate(slipAngleCurve, slipFront) * weightF,
	}
	lateralForceRear := rl.Vector2{
		X: 0,
		Y: FloatClamp(-5.2*slipRear, -2, 2) * weightR,
		// Y: LookupAndInterpolate(slipAngleCurve, slipRear) * weightR,
	}

	// if math.Abs(float64(car.Velocity.X)) < 0.1 {
	// 	lateralForceFront.X = 0
	// 	lateralForceFront.Y = 0
	// 	lateralForceRear.X = 0
	// 	lateralForceRear.Y = 0
	// }

	engineTorque := LookupAndInterpolate(car.Engine.TorqueCurve, car.Engine.Rpm) * car.Throttle

	tractionForce := rl.Vector2{
		X: Min(TractionForce(
			engineTorque,
			car.Transmission.Gears[car.Transmission.Gear],
			car.Transmission.DiffRatio,
			car.Transmission.Efficiency,
			car.WheelRadius,
		), weightR*car.WheelMu),
		Y: 0,
	}
	drag := Drag(car.FrictionCoeff, car.FrontalArea, airDensity)
	dragForce := rl.Vector2{
		X: -drag * car.Velocity.X * float32(math.Abs(float64(car.Velocity.X))),
		Y: -drag * car.Velocity.Y * float32(math.Abs(float64(car.Velocity.Y))),
	}
	brakeForce := rl.Vector2{
		X: -float32(Signum(car.Velocity.X)) * float32(7500*car.Brake),
		Y: 0,
	}
	rollingRes := rl.Vector2{
		X: -30 * drag * car.Velocity.X,
		Y: -30 * drag * car.Velocity.Y,
	}

	force := rl.Vector2{
		X: tractionForce.X + dragForce.X + brakeForce.X + rollingRes.X,
		Y: tractionForce.Y + dragForce.Y + brakeForce.Y + rollingRes.Y,
	}

	force.X += /*float32(math.Sin(float64(car.SteeringAngle)))* */ lateralForceFront.X + lateralForceRear.X
	force.Y += /*float32(math.Cos(float64(car.SteeringAngle)))* */ lateralForceFront.Y + lateralForceRear.Y

	torque := float32(math.Cos(float64(car.SteeringAngle)))*car.Wheelbase*0.5*lateralForceFront.Y - car.Wheelbase*0.5*lateralForceRear.Y

	car.Acceleration = rl.Vector2{
		X: force.X / car.Mass,
		Y: force.Y / car.Mass,
	}

	angularAcceleration := torque / car.Mass

	car.AngularVelocity += delta * angularAcceleration
	car.Rotation += delta * car.AngularVelocity

	if math.Abs(float64(car.Velocity.X)) < 0.1 {
		car.Velocity.X = 0
		car.AngularVelocity = 0
	}

	flywheelAccel := car.Engine.FlywheelAccel(engineTorque)
	if car.Transmission.Gear == 1 {
		car.Engine.Rpm += AngularVelocityToRpm(flywheelAccel) * 60 * delta

		if flywheelAccel <= 0 {
			car.Engine.Rpm -= 2000 * delta
		}
	} else {
		car.Engine.Rpm = EngineRpmFromWheelRpm(WheelRotationRateFromVelocity(
			car.Velocity.X, car.WheelRadius,
		), car.Transmission.Gears[car.Transmission.Gear], car.Transmission.DiffRatio)
	}
	if car.Engine.Rpm < car.Engine.MinRpm {
		car.Engine.Rpm = car.Engine.MinRpm
	}

	accelerationWc := rl.Vector2{
		X: cs*car.Acceleration.Y + sn*car.Acceleration.X,
		Y: -sn*car.Acceleration.Y + cs*car.Acceleration.X,
	}

	velocityWc.X += accelerationWc.X * delta
	velocityWc.Y += accelerationWc.Y * delta

	car.Velocity.X = cs*velocityWc.Y + sn*velocityWc.X
	car.Velocity.Y = -sn*velocityWc.Y + cs*velocityWc.X

	// theoretically := WheelSpeedmFromEngineRpm(
	// 	car.Engine.MinRpm,
	// 	car.Transmission.Gears[car.Transmission.Gear],
	// 	car.Transmission.DiffRatio,
	// 	car.Transmission.Efficiency,
	// 	car.WheelRadius,
	// )
	// if car.Transmission.Gear != 1 && math.Abs(float64(car.Velocity.X)) <= float64(theoretically) {
	// 	velocityWc.X = sn*theoretically
	// 	velocityWc.Y = cs*theoretically
	// }

	car.Position.X += delta * velocityWc.X
	car.Position.Y += delta * velocityWc.Y

	rl.DrawText(fmt.Sprintf(
		"alpha front  %8.6f deg"+
			"\nalpha rear   %8.6f deg"+
			"\nf.lat front  %8.2f N"+
			"\nf.lat rear   %8.2f N"+
			"\nforce.x      %8.2f N"+
			"\nforce.y lat  %8.2f N"+
			"\ntorque       %8.2f Nm"+
			"\nang.vel.     %8.2f rad/s"+
			"\nweight front %8.2f Nm"+
			"\nweight rear  %8.2f Nm",
		slipFront, slipRear,
		lateralForceFront.Y,
		lateralForceRear.Y,
		force.X,
		force.Y,
		torque,
		car.AngularVelocity,
		weightF,
		weightR,
		// car.Engine.Rpm, engineTorque, car.Transmission.Gear-1, velocity.X/1000*3600,
		// car.Rotation,
		// angularAcceleration,
		// lateralForceFront,
		// lateralForceRear,
	),
		10, 10, 20, rl.LightGray,
	)
}

func main() {
	rl.SetConfigFlags(rl.FlagMsaa4xHint)
	rl.InitWindow(800, 450, "raylib [core] example - basic window")

	rl.SetTargetFPS(60)
	var delta float32 = float32(1) / 60

	for !rl.WindowShouldClose() {
		rl.BeginDrawing()

		rl.ClearBackground(rl.RayWhite)

		Run(delta)

		if rl.IsKeyPressed(rl.KeyQ) && pixelsInMeter > 1 {
			pixelsInMeter -= 1
		}
		if rl.IsKeyPressed(rl.KeyW) {
			pixelsInMeter += 1
		}

		// rot1 := car.Rotation
		cs := float32(math.Cos(float64(car.Rotation)))
		sn := float32(math.Sin(float64(car.Rotation)))

		if car.Position.X*float32(pixelsInMeter) > float32(rl.GetScreenWidth()) {
			car.Position.X = 0
		}
		if car.Position.Y*float32(pixelsInMeter) > float32(rl.GetScreenHeight()) {
			car.Position.Y = 0
		}
		if car.Position.X*float32(pixelsInMeter) < 0 {
			car.Position.X = float32(rl.GetScreenWidth()) / float32(pixelsInMeter)
		}
		if car.Position.Y*8 < 0 {
			car.Position.Y = float32(rl.GetScreenHeight()) / float32(pixelsInMeter)
		}

		rect.X = car.Position.X * float32(pixelsInMeter)
		rect.Y = car.Position.Y * float32(pixelsInMeter)
		rect.Width = car.Dimensions.X * float32(pixelsInMeter)
		rect.Height = car.Dimensions.Y * float32(pixelsInMeter)

		wheelRect1 := wheelRect
		wheelRect1.Height = wheelRect.Height * float32(pixelsInMeter)
		wheelRect1.Width = wheelRect.Width * float32(pixelsInMeter)

		
		rot := -car.Rotation * 180 / math.Pi
		
		frontLeft := rl.Vector2{
			X: cs*car.Dimensions.X*0.5 + sn*car.Wheelbase*0.5,
			Y: -sn*car.Dimensions.X*0.5 + cs*car.Wheelbase*0.5,
		}
		frontRight := rl.Vector2{
			X: cs*(-car.Dimensions.X)*0.5 + sn*car.Wheelbase*0.5,
			Y: -sn*(-car.Dimensions.X)*0.5 + cs*car.Wheelbase*0.5,
		}
		rearLeft := rl.Vector2{
			X: cs*car.Dimensions.X*0.5 + sn*(-car.Wheelbase)*0.5,
			Y: -sn*car.Dimensions.X*0.5 + cs*(-car.Wheelbase)*0.5,
		}
		rearRight := rl.Vector2{
			X: cs*(-car.Dimensions.X)*0.5 + sn*(-car.Wheelbase)*0.5,
			Y: -sn*(-car.Dimensions.X)*0.5 + cs*(-car.Wheelbase)*0.5,
		}
		AddToTrail([]rl.Vector2{rl.Vector2Add(car.Position, rearLeft), rl.Vector2Add(car.Position, rearRight)})

		for _, v := range trail {
			for _, v2 := range v {
				rl.DrawCircle(
					int32(v2.X*float32(pixelsInMeter)),
					int32(v2.Y*float32(pixelsInMeter)),
					float32(pixelsInMeter) / 3.3, rl.LightGray,
				)
			}
		}
		rl.DrawRectanglePro(rect, rl.Vector2{X: rect.Width * 0.5, Y: rect.Height * 0.5}, rot, rl.Red)

		wheelRect1.X = float32(pixelsInMeter) * (car.Position.X + frontLeft.X)
		wheelRect1.Y = float32(pixelsInMeter) * (car.Position.Y + frontLeft.Y)
		rl.DrawRectanglePro(wheelRect1, rl.Vector2{X: wheelRect1.Width * 0.5, Y: wheelRect1.Height * 0.5}, rot-car.SteeringAngle*180/math.Pi, rl.Black)

		wheelRect1.X = float32(pixelsInMeter) * (car.Position.X + frontRight.X)
		wheelRect1.Y = float32(pixelsInMeter) * (car.Position.Y + frontRight.Y)
		rl.DrawRectanglePro(wheelRect1, rl.Vector2{X: wheelRect1.Width * 0.5, Y: wheelRect1.Height * 0.5}, rot-car.SteeringAngle*180/math.Pi, rl.Black)

		wheelRect1.X = float32(pixelsInMeter) * (car.Position.X + rearLeft.X)
		wheelRect1.Y = float32(pixelsInMeter) * (car.Position.Y + rearLeft.Y)
		rl.DrawRectanglePro(wheelRect1, rl.Vector2{X: wheelRect1.Width * 0.5, Y: wheelRect1.Height * 0.5}, rot, rl.Black)

		wheelRect1.X = float32(pixelsInMeter) * (car.Position.X + rearRight.X)
		wheelRect1.Y = float32(pixelsInMeter) * (car.Position.Y + rearRight.Y)
		rl.DrawRectanglePro(wheelRect1, rl.Vector2{X: wheelRect1.Width * 0.5, Y: wheelRect1.Height * 0.5}, rot, rl.Black)

		// var min float32 = 0
		var max float32 = 8000

		rl.DrawRing(rl.Vector2{
			X: float32(rl.GetScreenWidth()) - 100,
			Y: float32(rl.GetScreenHeight()) - 100,
		}, 30, 50,
			360*(max)/(max),
			360*(max-car.Engine.Rpm+2)/(max), 0, rl.Maroon,
		)

		rl.DrawRingLines(rl.Vector2{
			X: float32(rl.GetScreenWidth()) - 100,
			Y: float32(rl.GetScreenHeight()) - 100,
		}, 30, 50,
			360,
			1, 0, rl.Maroon,
		)

		rl.DrawRing(rl.Vector2{
			X: float32(rl.GetScreenWidth()) - 100,
			Y: float32(rl.GetScreenHeight()) - 140,
		}, 30, 50,
			180,
			180+2*car.SteeringAngle*180/math.Pi, 0, rl.Maroon,
		)

		rl.DrawRingLines(rl.Vector2{
			X: float32(rl.GetScreenWidth()) - 100,
			Y: float32(rl.GetScreenHeight()) - 140,
		}, 30, 50,
			-car.MaxSteeringAngle*2*180/math.Pi+180,
			car.MaxSteeringAngle*2*180/math.Pi+180, 0, rl.Maroon,
		)

		var barHeight float32 = 100
		rl.DrawRectangle(
			int32(rl.GetScreenWidth())-200,
			int32(rl.GetScreenHeight())-150+int32(barHeight-barHeight*car.Throttle),
			20, int32(barHeight*car.Throttle), rl.Green,
		)
		rl.DrawRectangleLines(
			int32(rl.GetScreenWidth())-200,
			int32(rl.GetScreenHeight())-150,
			20, int32(barHeight), rl.DarkGreen,
		)

		rl.DrawRectangle(
			int32(rl.GetScreenWidth())-230,
			int32(rl.GetScreenHeight())-150+int32(barHeight-barHeight*car.Brake),
			20, int32(barHeight*car.Brake), rl.Maroon,
		)
		rl.DrawRectangleLines(
			int32(rl.GetScreenWidth())-230,
			int32(rl.GetScreenHeight())-150,
			20, int32(barHeight), rl.Maroon,
		)

		rl.DrawText(
			fmt.Sprintf(
				"%d\n"+
					"%.0f\n"+
					"\n\n%.0f km/h",
				car.Transmission.Gear-1,
				car.Engine.Rpm,
				car.Velocity.X/1000*3600,
				// car.Engine.Rpm, engineTorque, car.Transmission.Gear-1, velocity.X/1000*3600,
				// car.Rotation,
				// angularAcceleration,
				// lateralForceFront,
				// lateralForceRear,
			),
			int32(rl.GetScreenWidth()-115),
			int32(rl.GetScreenHeight()-115), 14, rl.DarkGray,
		)

		rl.EndDrawing()
	}

	rl.CloseWindow()
}
