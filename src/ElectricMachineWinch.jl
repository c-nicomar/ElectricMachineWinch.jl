module ElectricMachineWinch

using WinchModels
using IM_AWES_bench

const IMB = IM_AWES_bench

export DetailedIMWinch
export InductionMachinePlant
export DriveStepOutput
export AbstractIMDriveController
export IdealTorqueController
export FOCSpeedF1Controller
export drive_step!
export make_default_foc_speed_controller
export make_electric_winch
export reset!
export last_summary

include("types.jl")
include("plant_steps.jl")
include("controllers/ideal_torque.jl")
include("controllers/foc_speed_f1.jl")
include("winch_interface.jl")
include("constructors.jl")

end # module
