# activate the test environment if needed
using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using Timers; tic()

using ElectricMachineWinch

if !@isdefined __PRECOMPILE__
    __PRECOMPILE__ = false
end

LOG_LIFT_DRAG::Bool = false
DRAG_CORR::Float64 = 0.93 

using KiteViewers
using KiteControllers, KiteModels, MakieControlPlots, NativeFileDialog, LaTeXStrings, Statistics
using LinearAlgebra, Printf, DelimitedFiles
using KiteViewers: Viewer3D
import KiteViewers.GLMakie
import KiteViewers.GLMakie.GLFW

# KiteControllers only auto-detects `pwd()/data` as the data path when it
# contains `system.yaml`, which this repo's `data/` does not ship. Point it
# explicitly at this repo's `data/` directory so gui.yaml/read_project() work
# regardless of the current working directory or whether install_yaml_files.jl
# already ran in this Julia session.
set_data_path(joinpath(@__DIR__, "..", "data"))

# read_project() is provided by KiteControllers.
PROJECT = read_project()
GLMakie.activate!(title = PROJECT)
OUTPUT_DIR::String = "output"
mkpath(OUTPUT_DIR)
@assert isdir(OUTPUT_DIR)
DEFAULT_LOG::String = joinpath(OUTPUT_DIR, "last_sim_log")

# -------------------------------------------------------------------------
# ElectricMachineWinch debug log
#
# This is an in-memory log independent of the standard KiteControllers logger.
# It is filled once per outer autopilot step, after the electric drive
# has been advanced internally and before KiteModels.next_step! is called.
# After the simulation, call:
#   print_emw_summary()
#   plot_emw_speed()
#   plot_emw_machine_speed()
#   plot_emw_torque()
#   plot_emw_currents()
#   plot_emw_voltages()
#   plot_emw_power()
#   save_emw_debug_csv()
# -------------------------------------------------------------------------
const EMW_DEBUG_LOG = Vector{NamedTuple}()


function test_observer(plot=true)
    log = load_log("uncorrected")
    ob = KiteObserver()
    observe!(ob, log)
    if plot
        plotxy(ob.fig8, ob.elevation, xlabel="fig8", ylabel="elevation")
    else
        ob
    end
end

mutable struct KiteApp
    set::Settings
    max_time::Float64
    next_max_time::Float64
    show_kite::Bool
    kcu::Union{KCU, Nothing} 
    kps4::Union{KPS4, Nothing}
    wcs::Union{WCSettings, Nothing}
    fcs::Union{FPCSettings, Nothing}
    fpps::Union{FPPSettings, Nothing}
    ssc::Union{SystemStateControl, Nothing}
    viewer::Union{Viewer3D, Nothing}
    logger::Union{Logger, Nothing}
    dt::Float64
    steps::Int64 # simulation steps for one simulation
    particles::Int64
    run::Int64
    parking::Bool
    initialized::Bool
end
app::KiteApp = KiteApp(deepcopy(load_settings(PROJECT)), 0, 0, true, nothing, nothing, nothing,
                       nothing, nothing, nothing, nothing, nothing, 0, 0, 0, 0, false, false)
let use_turbulence = KiteControllers.get_use_turbulence(PROJECT)
    isnothing(use_turbulence) || (app.set.use_turbulence = use_turbulence)
end
app.max_time      = app.set.sim_time
app.next_max_time = app.max_time
const MENU_REQUEST = Ref{Union{Nothing,String}}(nothing)

function init(app::KiteApp; init_viewer=false)
    app.max_time = app.next_max_time
app.kcu = KCU(app.set)
project = KiteUtils.PROJECT

app.wcs = WCSettings(true; dt=1/app.set.sample_freq)
app.wcs.dt = 1/app.set.sample_freq
app.dt = app.wcs.dt
app.kps4 = KPS4(app.kcu::KCU)
app.kps4.wm = make_electric_winch(
    controller = :foc_speed_f1,

    drum_radius = 0.2,
    gear_ratio = 4.26,
    J_eq = 0.204,
    B_eq = 0.0804,

    Ts = 1e-4,
    plant_substeps = 1,

    Vs_max = 310.0,
    Is_max = 40.0*sqrt(2),
    Te_max = 124.0,

    speed_ts_wm = 1.0,
    use_load_feedforward = true,

    use_field_weakening = true,
    wm_base_fw = 120.0,
)

KiteUtils.PROJECT = project
    app.wcs = WCSettings(true; dt=1/app.set.sample_freq)
    
    app.wcs.dt = 1/app.set.sample_freq
    app.dt = app.wcs.dt
    app.fcs = FPCSettings(true; dt=app.dt) 
    app.fcs.log_level = app.set.log_level
    app.fpps = FPPSettings(true)
    app.fpps.log_level = app.set.log_level
    u_d0 = 0.01 * se(project).depower_offset
    u_d = 0.01 * se(project).depowers[1]
    app.ssc = SystemStateControl(app.wcs, app.fcs, app.fpps; u_d0, u_d, v_wind=app.set.v_wind)
    if init_viewer
        app.viewer= Viewer3D(app.set, app.show_kite; menus=true)
        app.viewer.menu.options[]=["plot_main", "plot_power", "plot_control", "plot_control_II", "plot_winch_control",
                                   "plot_emw_speed_torque", "plot_emw_currents_power", "print_emw_summary",
                                   "save_emw_debug_csv", "plot_aerodynamics",
                                   "plot_elev_az", "plot_elev_az2", "plot_elev_az3", "plot_side_view", "plot_side_view2", "plot_side_view3", "plot_front_view3", "plot_timing", 
                                   "print_stats", "load logfile", "save logfile"]
        app.viewer.menu_rel_tol.options[]=["0.005","0.001","0.0005","0.0001","0.00005", "0.00001",
                                           "0.000005","0.000001"]
        app.viewer.menu_time_lapse.options[]=[ "1x","2x","3x","4x","6x","9x","12x","18x","24x"]
        app.viewer.menu_project.options[]=["Open...", "Save as...", "Edit..."]
    end
    if app.set.time_lapse==24.0
        app.viewer.menu_time_lapse.i_selected[] = 9
    elseif app.set.time_lapse==18.0
        app.viewer.menu_time_lapse.i_selected[] = 8
    elseif app.set.time_lapse==12.0
        app.viewer.menu_time_lapse.i_selected[] = 7    
    elseif app.set.time_lapse==9.0
        app.viewer.menu_time_lapse.i_selected[] = 6
    elseif app.set.time_lapse==6.0
        app.viewer.menu_time_lapse.i_selected[] = 5
    elseif app.set.time_lapse==4.0
        app.viewer.menu_time_lapse.i_selected[] = 4
    elseif app.set.time_lapse==3.0
        app.viewer.menu_time_lapse.i_selected[] = 3
    elseif app.set.time_lapse==2.0
        app.viewer.menu_time_lapse.i_selected[] = 2
    elseif app.set.time_lapse==1.0
        app.viewer.menu_time_lapse.i_selected[] = 1
    else
        println("Warning: Invalid setting for time_lapse in config file.")
    end
    app.viewer.t_sim.displayed_string[]=repr(Int64(round(app.set.sim_time)))
    app.steps = Int64(app.max_time/app.dt)
    app.particles = app.set.segments + 5
    app.logger = Logger(app.particles, app.steps)
    app.parking = false
    app.max_time      = app.set.sim_time
    app.next_max_time = app.max_time
    app.initialized = true
end

# the following values can be changed to match your interest
DEFAULT_TOLERANCE = 3
# end of user parameter section #

init(app; init_viewer=true)
bring_viewer_to_front()

function simulate(integrator, stopped=true)
    start_time_ns = time_ns()
    sys_state = SysState(app.kps4::KPS4)
    sys_state.e_mech = 0
    sys_state.sys_state = Int16(app.ssc.fpp._state)
    e_mech = 0.0
    last_vel = [0.0, 0.0, 0.0]
    on_new_systate(app.ssc::SystemStateControl, sys_state)
    clear_viewer(app.viewer::Viewer3D)
    KiteViewers.update_system(app.viewer::Viewer3D, sys_state; scale = 0.04/1.1, kite_scale=app.set.kite_scale)
    KiteViewers.running[] = ! stopped
    app.viewer.stop = stopped
    if ! stopped
        set_status(app.viewer::Viewer3D, "ssParking")
    end
    i=1
    j=0; k=0
    GC.enable(true)
    GC.gc()
    mem_start=Sys.total_memory()/1e9 
    if Sys.total_memory()/1e9 > 24 && app.max_time < 1002
        GC.enable(false)
    end
    max_time = 0
    last_yaw = 0.0
    last_yaw_rate = 0.0
    while app.initialized
        if Sys.isapple() && MENU_REQUEST[] !== nothing
            c = MENU_REQUEST[]
            MENU_REQUEST[] = nothing
            if is_plot_menu_action(c)
                app.viewer.stop = true
                KiteViewers.running[] = false
                sleep(0.02)
            end
            do_menu(c)
        end
        local last_Te_cmd = 0.0
        if app.viewer.stop
            sleep(app.dt)
        else
            if i == 1
                app.max_time = app.next_max_time
                app.steps = Int64(app.max_time/app.dt)
                app.particles = app.set.segments + 5
                app.logger = Logger(app.particles, app.steps)
                log!(app.logger::Logger, sys_state)
                saved_use_turbulence = app.set.use_turbulence
                app.set.use_turbulence = 0.0
                try
                    integrator = init!(app.kps4::KPS4; delta=app.set.delta, stiffness_factor=app.set.stiffness_factor)
                finally
                    app.set.use_turbulence = saved_use_turbulence
                end
            end
            if mod(i, 100) == 0 && app.set.log_level > 0
                println("Free memory: $(round(Sys.free_memory()/1e9, digits=1)) GB") 
            end
            if i > 100
                dp = KiteControllers.get_depower(app.ssc::SystemStateControl)
                if dp < 0.22 dp = 0.22 end
                heading = calc_heading(app.kps4::KPS4; neg_azimuth=true, one_point=false)
                azimuth = -calc_azimuth(app.kps4::KPS4)
                steering = -calc_steering(app.ssc::SystemStateControl; heading, azimuth)
                set_depower_steering((app.kps4::KPS4).kcu, dp, steering)
            end
            if i == 200 && ! app.parking
                on_autopilot(app.ssc::SystemStateControl)
            end
            # execute winch controller
            v_ro_set = calc_v_set(app.ssc::SystemStateControl)

            F_tether = sys_state.winch_force[1]
            v_ro_meas = sys_state.v_reelout[1]

            Te_cmd = ElectricMachineWinch.step_drive_from_kite!(
                app.kps4.wm,
                v_ro_set,
                v_ro_meas,
                F_tether;
                dt_outer = app.dt,
            )

            last_Te_cmd = Te_cmd

            # -------------------------------------------------------------
            # ElectricMachineWinch debug sampling
            # -------------------------------------------------------------
            # Log one row per autopilot macro-step. The FOC/electrical
            # controller may have run many internal substeps inside
            # step_drive_from_kite!, but the row below stores the final values
            # that are seen by the kite model at this macro-step.
            if app.kps4.wm isa ElectricMachineWinch.DetailedIMWinch
                ew = app.kps4.wm
                R = ew.drum_radius
                n = ew.gear_ratio

                ωm_set = n / R * Float64(v_ro_set)
                ωm_meas = n / R * Float64(v_ro_meas)
                Tload = R / n * Float64(F_tether)
                n_drive_substeps = max(1, round(Int, app.dt / ew.Ts))

                # Stator currents in abc from alpha-beta electrical plant state.
                ia = ew.plant.x.isα
                ib = -0.5 * ew.plant.x.isα + sqrt(3) / 2 * ew.plant.x.isβ
                ic = -0.5 * ew.plant.x.isα - sqrt(3) / 2 * ew.plant.x.isβ

                # Stator phase voltages in abc from alpha-beta FOC voltage command.
                va = ew.vsα
                vb = -0.5 * ew.vsα + sqrt(3) / 2 * ew.vsβ
                vc = -0.5 * ew.vsα - sqrt(3) / 2 * ew.vsβ

                # Two useful power definitions.
                # Pmech_tether is kite/drum-side mechanical power.
                # Pelec_abc is stator electrical power computed from abc values.
                Pmech_tether = Float64(F_tether) * Float64(v_ro_meas)
                Pelec_abc = va * ia + vb * ib + vc * ic

                push!(EMW_DEBUG_LOG, (
                    t = (i - 1) * app.dt,

                    # Kite-side reel-out speed [m/s]
                    v_ro_set = Float64(v_ro_set),
                    v_ro_meas = Float64(v_ro_meas),
                    v_ro_error = Float64(v_ro_set - v_ro_meas),

                    # Machine-side speed [rad/s]
                    wm_set = ωm_set,
                    wm_meas = ωm_meas,
                    wm_error = ωm_set - ωm_meas,

                    # Force and torque
                    F_tether = Float64(F_tether),
                    Tload = Tload,
                    Te_cmd = Float64(Te_cmd),
                    Te = ew.Te,
                    Te_ref = ew.Te_ref,
                    Te_error = ew.Te_ref - ew.Te,

                    # dq currents
                    isd = ew.isd,
                    isq = ew.isq,
                    isd_ref = ew.isd_ref,
                    isq_ref = ew.isq_ref,
                    isd_error = ew.isd_ref - ew.isd,
                    isq_error = ew.isq_ref - ew.isq,

                    # dq voltages
                    vsd = ew.vsd,
                    vsq = ew.vsq,

                    # abc stator currents [A]
                    ia = ia,
                    ib = ib,
                    ic = ic,

                    # abc stator voltages [V]
                    va = va,
                    vb = vb,
                    vc = vc,

                    # alpha-beta plant currents and rotor currents
                    isα = ew.plant.x.isα,
                    isβ = ew.plant.x.isβ,
                    irα = ew.plant.x.irα,
                    irβ = ew.plant.x.irβ,

                    # Powers
                    Pelec = ew.Pelec,
                    Pelec_abc = Pelec_abc,
                    Pmech = ew.Pmech,
                    Pmech_tether = Pmech_tether,

                    # Status and timing
                    saturated = ew.saturated ? 1.0 : 0.0,
                    drive_Ts = ew.Ts,
                    outer_dt = app.dt,
                    n_drive_substeps = Float64(n_drive_substeps),
                    n_acceleration_calls = Float64(ew.n_acceleration_calls),
                ))
            end

            t_sim = @elapsed KiteModels.next_step!(
                app.kps4::KPS4,
                integrator;
                set_torque = Te_cmd,
                dt = app.dt,
            )
            KiteModels.update_sys_state!(sys_state, app.kps4::KPS4)
            acc = ((app.kps4::KPS4).vel_kite - last_vel)/app.dt
            last_vel = deepcopy((app.kps4::KPS4).vel_kite)

            on_new_systate(app.ssc::SystemStateControl, sys_state)
            e_mech += (sys_state.winch_force[1] * sys_state.v_reelout[1])/3600*app.dt
            sys_state.e_mech = e_mech
            sys_state.sys_state = Int16(app.ssc.fpp._state)
            sys_state.cycle  = app.ssc.fpp.fpca.cycle
            sys_state.fig_8   = app.ssc.fpp.fpca.fig8
            sys_state.var_03 = get_state(app.ssc.wc) # 0=lower_force_control 1=square_root_control 2=upper_force_control
            sys_state.var_04 = app.ssc.wc.lfc.f_set # set force of lower force controller
            sys_state.var_05 = app.ssc.wc.lfc.v_set_out
            sys_state.var_06 = app.ssc.fpp.fpca.fpc.ndi_gain
            if isnothing(app.ssc.fpp.fpca.fpc.psi_dot_set)
                sys_state.var_07 = app.ssc.fpp.fpca.fpc.chi_set
                sys_state.var_10 = NaN
                sys_state.var_09 = NaN
            else
                sys_state.var_07 = NaN
                sys_state.var_09 = app.ssc.fpp.fpca.fpc.psi_dot_set
                sys_state.var_10 = app.ssc.fpp.fpca.fpc.est_psi_dot
            end
            
            sys_state.var_11 = app.ssc.fpp.fpca.fpc.est_chi_dot
            sys_state.var_12 = app.ssc.fpp.fpca.fpc.c2
            sys_state.acc = norm(acc)
            if abs((sys_state.yaw - last_yaw) / app.dt ) < 20.0
                sys_state.var_15 = (sys_state.yaw - last_yaw) / app.dt # yaw rate
            else
                sys_state.var_15 = last_yaw_rate
            end
            last_yaw = sys_state.yaw
            last_yaw_rate = sys_state.var_15
            sys_state.var_16 = app.kps4.side_slip
            
            sys_state.var_08 = norm(app.kps4.lift_force)/norm(app.kps4.drag_force)
            if i > 10
                sys_state.t_sim = t_sim*1000
            end
            log!(app.logger::Logger, sys_state)
            if mod(app.set.time_lapse, 3) == 0
                ratio = 3
            elseif mod(app.set.time_lapse, 2) == 0
                ratio = 2
            else
                ratio = 1
            end
            if app.set.time_lapse == 12
                ratio = 4
            end
            app.viewer.mod_text = 3*ratio
            if mod(i, Int64(app.set.time_lapse)/ratio) == 0 
                KiteViewers.update_system(app.viewer::Viewer3D, sys_state; scale = 0.04/1.1, kite_scale=app.set.kite_scale)
                set_status(app.viewer::Viewer3D, String(Symbol(app.ssc.state)))
                # re-enable garbage collector when we are short of memory
                if Sys.free_memory()/1e9 < 4.0
                    GC.enable(true)
                end
                wait_until(start_time_ns + 1e9*app.dt/ratio, always_sleep=true) 
                mtime = 0
                if i > 10/app.dt 
                    # if we missed the deadline by more than 1 ms
                    mtime = time_ns() - start_time_ns
                    if mtime > app.dt*1e9/ratio + 1e6
                        print(".")
                        j += 1
                    end
                    k +=1
                end
                if mtime > max_time
                    max_time = mtime
                end            
                start_time_ns = time_ns()
            end
            i += 1
        end
        if ! isopen(app.viewer.fig.scene) break end
        if KiteViewers.status[] == "Stopped" && i > 10 
            if app.set.log_level > 0
                @timev KiteModels.next_step!(app.kps4::KPS4, integrator; set_torque=last_Te_cmd, dt=app.dt)
            else
                KiteModels.next_step!(app.kps4::KPS4, integrator; set_torque=last_Te_cmd, dt=app.dt)
            end
            break 
        end
        if i*app.dt > app.max_time break end
    end
    mem_used=mem_start-Sys.free_memory()/1e9 
    if app.set.log_level > 0
        println("\nMaximal memory usage: $(round(mem_used, digits=1)) GB")
    end
    if i > 10/app.dt
        misses = j/k * 100
        println("\nMissed the deadline for $(round(misses, digits=2)) %. Max time: $(round((max_time*1e-6), digits=1)) ms")
    end
    return div(i, Int64(app.set.time_lapse))
end

function play(stopped=false)
    while isopen(app.viewer.fig.scene)
        if ! app.initialized
            init(app)
        end
        KiteViewers.plot_file[]=DEFAULT_LOG
        on_parking(app.ssc::SystemStateControl)
        saved_use_turbulence = app.set.use_turbulence
        app.set.use_turbulence = 0.0
        integrator = try
            init!(app.kps4::KPS4; delta=app.set.delta, stiffness_factor=app.set.stiffness_factor)
        finally
            app.set.use_turbulence = saved_use_turbulence
        end
        if !isnothing(app.viewer)
            _ss = SysState(app.kps4::KPS4)
            _ss.sys_state = Int16(app.ssc.fpp._state)
            KiteViewers.update_system(app.viewer::Viewer3D, _ss; scale = 0.04/1.1, kite_scale=app.set.kite_scale)
        end
        if app.run == 0; toc(); end
        app.run += 1
        simulate(integrator, stopped)
        app.initialized = false
        stopped = ! app.viewer.sw.active[]
        if !isnothing(app.logger) && (app.logger::Logger).index > 100
            KiteViewers.plot_file[]=DEFAULT_LOG
            if app.set.log_level > 0
                println("Saving log... $((app.logger::Logger).index)")
            end
            save_log(app.logger::Logger, basename(DEFAULT_LOG); path=dirname(DEFAULT_LOG))
        end
        if (@isdefined __PRECOMPILE__) && __PRECOMPILE__
            break
        end
    end
    GC.enable(true)
end

function parking()
    app.parking     = true
    app.viewer.stop = false
    on_parking(app.ssc::SystemStateControl)
end

function autopilot()
    app.parking     = false
    app.viewer.stop = false
    on_autopilot(app.ssc::SystemStateControl)
end

function stop_(; clear_display=true)
    if app.set.log_level > 0
        println("Stopping...")
    end
    on_stop(app.ssc::SystemStateControl)
    clear!(app.kps4::KPS4)
    if clear_display && ! isnothing(app.viewer)
        clear_viewer(app.viewer::Viewer3D)
    end
end

stop_(; clear_display=false)
on(app.viewer.btn_PARKING.clicks) do _; parking(); end
on(app.viewer.btn_AUTO.clicks) do _; autopilot(); end
on(app.viewer.btn_STOP.clicks) do _; stop_(); end
on(app.viewer.btn_PLAY.clicks) do _;
    if ! app.viewer.stop
        app.parking = false
    end
end
on(app.viewer.menu_time_lapse.selection) do _;
    val=app.viewer.menu_time_lapse.selection[][begin:end-1]
    app.set.time_lapse=parse(Int64, val)
end

# GTK3 (used by NativeFileDialog on Linux) prints warnings directly to fd 2;
# redirect at the OS level to suppress them during file dialogs.
function without_gtk_warnings(f)
    if !Sys.islinux()
        return f()
    end
    old_fd = ccall(:dup, Cint, (Cint,), 2)
    null_fd = ccall(:open, Cint, (Cstring, Cint), "/dev/null", 1)
    ccall(:dup2, Cint, (Cint, Cint), null_fd, 2)
    ccall(:close, Cint, (Cint,), null_fd)
    try
        return f()
    finally
        ccall(:dup2, Cint, (Cint, Cint), old_fd, 2)
        ccall(:close, Cint, (Cint,), old_fd)
    end
end

function select_log()
    @async begin 
        filename = without_gtk_warnings() do
            pick_file("output"; filterlist="arrow")
        end
        if filename != ""
            short_filename = replace(filename, homedir() => "~")
            KiteViewers.plot_file[] = short_filename
        end
    end
end

function save_log_as()
    @async begin 
        filename = without_gtk_warnings() do
            save_file("output"; filterlist="arrow")
        end
        if filename != ""
            source = replace(KiteViewers.plot_file[], "~" => homedir()) * ".arrow"
            if ! isfile(source)
                source = joinpath(pwd(), "output", KiteViewers.plot_file[]) * ".arrow"
            end
            dest  = filename
            if app.set.log_level > 0
                println("Copying: ", source, " => ", dest)
            end
            cp(source, dest; force=true)
            KiteViewers.set_status(app.viewer, "Saved log as:")
            KiteViewers.plot_file[] = replace(filename, homedir() => "~")
        end
    end
end

const KC_EXAMPLES_DIR = joinpath(dirname(pathof(KiteControllers)), "..", "examples")

include(joinpath(KC_EXAMPLES_DIR, "plots.jl"))
include(joinpath(KC_EXAMPLES_DIR, "stats.jl"))
include(joinpath(KC_EXAMPLES_DIR, "yaml_utils.jl"))

function show_stats(stats::Stats)
    HEIGHT = 440
    UPPER_BORDER = 20
    fig  = GLMakie.Figure(size = (400, HEIGHT))
    font = if Sys.islinux()
        # sudo apt install ttf-bitstream-vera
        lin_font = "/usr/share/fonts/truetype/ttf-bitstream-vera/VeraMono.ttf"
        isfile(lin_font) ? lin_font : "/usr/share/fonts/truetype/freefont/FreeMono.ttf"
    else
        "Courier New"
    end
    function print(lbl::String, value::String; line, font=font)
        GLMakie.text!(fig.scene, 20, HEIGHT-UPPER_BORDER-line*32; text=lbl, fontsize = 24, space=:pixel)
        GLMakie.text!(fig.scene, 250, HEIGHT-UPPER_BORDER-line*32; text=value, fontsize = 24, font, space=:pixel)
        line +=1    
    end
    line = print("energy:       ", @sprintf("%5.0f Wh", stats.e_mech); line = 1)
    line = print("average power:", @sprintf("%5.0f  W", stats.av_power); line)
    line = print("peak power:", @sprintf("%5.0f  W", stats.peak_power); line)
    line = print("min force:    ", @sprintf("%5.0f  N", stats.min_force); line)
    line = print("max force:    ", @sprintf("%5.0f  N", stats.max_force); line)
    line = print("min height:   ", @sprintf("%5.0f  m", stats.min_height); line)
    line = print("max height:   ", @sprintf("%5.0f  m", stats.max_height); line)
    line = print("min elevation:", @sprintf("%5.1f  °", stats.min_elevation); line)
    line = print("max elev_ro:  ", @sprintf("%5.1f  °", stats.max_elev_ro); line)
    line = print("min az_ro:    ", @sprintf("%5.1f  °", stats.min_az_ro); line)
    line = print("max az_ro:    ", @sprintf("%5.1f  °", stats.max_az_ro); line)
    line = print("cycles:       ", @sprintf("%5d   ", stats.cycles); line)
    print("turb. int.:   ", @sprintf("%5.1f  %%", stats.ti); line)

    display(GLMakie.Screen(), fig)
    nothing
end

function print_stats()
    log_file_exists() || return
    lg = load_log(basename(KiteViewers.plot_file[]); path=dirname(KiteViewers.plot_file[]))
    sl  = lg.syslog
    elev_ro = deepcopy(sl.elevation)
    az_ro = deepcopy(sl.azimuth)
    for i in eachindex(sl.sys_state)
        if ! (sl.sys_state[i] in (5,6,7,8))
            elev_ro[i] = 0
            az_ro[i] = 0
        end
    end
    av_power = 0.0
    peak_power = 0.0
    n = 0
    last_full_cycle = maximum(sl.cycle)-1
    force_ = force(sl)
    v_reelout_ = v_reelout(sl)
    for i in eachindex(force_)
        if sl.cycle[i] in 2:last_full_cycle
            av_power += force_[i] * v_reelout_[i]
            n+=1
        end
        if abs(force_[i] * v_reelout_[i]) > peak_power
            peak_power = abs(force_[i] * v_reelout_[i])
        end
    end
    av_power /= n
    v_wind_kite_norm = norm.(sl.v_wind_kite)
    v = filter(!=(0.0f0), v_wind_kite_norm)
    ti = std(v) / mean(v) * 100
    stats = Stats(sl[end].e_mech, av_power, peak_power, minimum(force_[Int64(round(5/app.dt)):end]), maximum(force_),
                  minimum(lg.z), maximum(lg.z), minimum(rad2deg.(sl.elevation)), maximum(rad2deg.(elev_ro)),
                  minimum(rad2deg.(az_ro)), maximum(rad2deg.(az_ro)), last_full_cycle, ti)
    show_stats(stats)
end

function is_plot_menu_action(c::AbstractString)
    startswith(c, "plot_") || c == "print_stats" || c == "print_emw_summary" || c == "save_emw_debug_csv"
end

function queue_or_execute_menu_action(c::AbstractString)
    if Sys.isapple()
        MENU_REQUEST[] = c
    else
        do_menu(c)
    end
    nothing
end

function do_menu(c)
    if isnothing(app.viewer) || !isopen(app.viewer.fig.scene)
        return nothing
    end
    if c == "save logfile"
        save_log_as()
    elseif c == "load logfile"
        select_log()
    elseif c == "plot_timing"
        plot_timing()
    elseif c == "plot_power"
        plot_power()
    elseif c == "plot_control"
        plot_control()
    elseif c == "plot_control_II"
        plot_control_II()
    elseif c == "plot_winch_control"
        plot_winch_control()
    elseif c == "plot_emw_speed_torque"
        plot_emw_speed_torque()
    elseif c == "plot_emw_currents_power"
        plot_emw_currents_power()
    elseif c == "print_emw_summary"
        print_emw_summary()
    elseif c == "save_emw_debug_csv"
        save_emw_debug_csv()
    elseif c == "plot_aerodynamics"
        plot_aerodynamics(LOG_LIFT_DRAG)
    elseif c == "plot_elev_az"
        plot_elev_az()
    elseif c == "plot_elev_az2"
        plot_elev_az2()
    elseif c == "plot_elev_az3"
        plot_elev_az3()
    elseif c == "plot_main"
        plot_main()
    elseif c == "plot_side_view"
        plot_side_view()
    elseif c == "plot_side_view2"
        plot_side_view2()
    elseif c == "plot_side_view3"
        plot_side_view3()
    elseif c == "plot_front_view3"
        plot_front_view3()
    elseif c == "print_stats"
        print_stats()
    end
end

on(app.viewer.btn_OK.clicks) do _
    queue_or_execute_menu_action(app.viewer.menu.selection[])
end

on(app.viewer.menu.selection) do c
    queue_or_execute_menu_action(c)
end

on(app.viewer.menu_rel_tol.selection) do c
    rel_tol = parse(Float64, c)
    factor = rel_tol/0.001
    app.set.rel_tol = rel_tol
    app.set.abs_tol = factor * 0.0006 
end

on(app.viewer.menu_project.i_selected) do _
    global PROJECT, app
    sel = app.viewer.menu_project.selection[]
    if sel == "Open..."
        @async begin 
            filename = without_gtk_warnings() do
                pick_file("data"; filterlist="yml")
            end
            if filename != ""
                PROJECT = basename(filename)
                GLFW.SetWindowTitle(app.viewer.screen.glscreen, PROJECT)
                lines = readfile(joinpath(KiteControllers.KiteUtils.get_data_path(), "gui.yaml"))
                lines = change_value(lines, "project:", PROJECT)
                writefile(lines, joinpath(KiteControllers.KiteUtils.get_data_path(), "gui.yaml"))
                sleep(0.1)
                app.set = deepcopy(load_settings(PROJECT))
                app.max_time      = app.set.sim_time
                app.next_max_time = app.max_time
                app.initialized = false
            end
        end
    end
end

on(app.viewer.t_sim.stored_string) do c
    val = (parse(Int64, c))
    if val == 0
        val = repr(Int64(round(app.set.sim_time)))
        app.viewer.t_sim.displayed_string[]=repr(Int64(round(val)))
    end
    app.next_max_time=val
    app.set.sim_time=val
end


# =============================================================================
# ElectricMachineWinch debug helpers
# =============================================================================

function emw_has_data()
    if isempty(EMW_DEBUG_LOG)
        println("EMW_DEBUG_LOG is empty. Run the simulation first.")
        return false
    end
    return true
end

function emw_series(name::Symbol)
    return [getfield(row, name) for row in EMW_DEBUG_LOG]
end

function _emw_plot(title::String, xlabel::String, ylabel::String, series::Pair{String, Vector{Float64}}...)
    emw_has_data() || return nothing

    t = emw_series(:t)

    fig = GLMakie.Figure(size = (900, 520))
    ax = GLMakie.Axis(fig[1, 1], title = title, xlabel = xlabel, ylabel = ylabel)

    for s in series
        GLMakie.lines!(ax, t, s.second, label = s.first)
    end

    GLMakie.axislegend(ax, position = :rb)
    display(GLMakie.Screen(), fig)
    return fig
end


"""
    plot_emw_speed_torque()

One compact figure with two subplots:
1. commanded and measured machine rotational speed [rad/s]
2. load torque, electromagnetic torque reference, and actual electromagnetic torque [Nm]
"""
function plot_emw_speed_torque()
    emw_has_data() || return nothing

    t = emw_series(:t)

    fig = GLMakie.Figure(size = (1000, 720))

    ax1 = GLMakie.Axis(
        fig[1, 1],
        title = "Machine rotational speed",
        xlabel = "Time [s]",
        ylabel = "ωm [rad/s]",
    )
    GLMakie.lines!(ax1, t, emw_series(:wm_set), label = "ωm reference")
    GLMakie.lines!(ax1, t, emw_series(:wm_meas), label = "ωm actual")
    GLMakie.axislegend(ax1, position = :rb)

    ax2 = GLMakie.Axis(
        fig[2, 1],
        title = "Load and electromagnetic torque",
        xlabel = "Time [s]",
        ylabel = "Torque [Nm]",
    )
    GLMakie.lines!(ax2, t, emw_series(:Tload), label = "Tload")
    GLMakie.lines!(ax2, t, emw_series(:Te_ref), label = "Te reference")
    GLMakie.lines!(ax2, t, emw_series(:Te), label = "Te actual")
    GLMakie.axislegend(ax2, position = :rb)

    display(GLMakie.Screen(), fig)
    return fig
end

"""
    plot_emw_currents_power()

One compact figure with two subplots:
1. dq current references and measured/observed dq currents [A]
2. tether mechanical power F_tether*v_ro and stator electrical power va*ia+vb*ib+vc*ic [W]

The abc currents are still logged for CSV/debugging, but they are not plotted here
because the outer autopilot sampling period is too slow to make phase
waveforms visually meaningful.
"""
function plot_emw_currents_power()
    emw_has_data() || return nothing

    t = emw_series(:t)

    fig = GLMakie.Figure(size = (1000, 720))

    ax1 = GLMakie.Axis(
        fig[1, 1],
        title = "FOC dq currents",
        xlabel = "Time [s]",
        ylabel = "Current [A]",
    )
    GLMakie.lines!(ax1, t, emw_series(:isd_ref), label = "isd reference")
    GLMakie.lines!(ax1, t, emw_series(:isd), label = "isd measured")
    GLMakie.lines!(ax1, t, emw_series(:isq_ref), label = "isq reference")
    GLMakie.lines!(ax1, t, emw_series(:isq), label = "isq measured")
    GLMakie.axislegend(ax1, position = :rb)

    ax2 = GLMakie.Axis(
        fig[2, 1],
        title = "Mechanical and electrical power",
        xlabel = "Time [s]",
        ylabel = "Power [W]",
    )
    GLMakie.lines!(ax2, t, emw_series(:Pmech_tether), label = "Pmech = F_tether*v_ro")
    GLMakie.lines!(ax2, t, -emw_series(:Pelec_abc), label = "-Pelec = -(va*ia+vb*ib+vc*ic)")
    GLMakie.axislegend(ax2, position = :rb)

    display(GLMakie.Screen(), fig)
    return fig
end

"""
    plot_emw_speed()

Plot commanded and measured reel-out speed in m/s.
"""
function plot_emw_speed()
    emw_has_data() || return nothing
    _emw_plot(
        "Electric winch reel-out speed",
        "Time [s]",
        "Reel-out speed [m/s]",
        "v_ro_set" => emw_series(:v_ro_set),
        "v_ro_meas" => emw_series(:v_ro_meas),
        "v_ro_error" => emw_series(:v_ro_error),
    )
end

"""
    plot_emw_machine_speed()

Plot commanded and measured machine-side angular speed in rad/s.
"""
function plot_emw_machine_speed()
    emw_has_data() || return nothing
    _emw_plot(
        "Electric machine angular speed",
        "Time [s]",
        "Machine speed [rad/s]",
        "ωm_set" => emw_series(:wm_set),
        "ωm_meas" => emw_series(:wm_meas),
        "ωm_error" => emw_series(:wm_error),
    )
end

"""
    plot_emw_torque()

Plot load torque, electromagnetic torque reference and produced torque.
"""
function plot_emw_torque()
    emw_has_data() || return nothing
    _emw_plot(
        "Electric winch torque variables",
        "Time [s]",
        "Torque [Nm]",
        "Tload" => emw_series(:Tload),
        "Te_ref" => emw_series(:Te_ref),
        "Te" => emw_series(:Te),
        "Te_cmd" => emw_series(:Te_cmd),
    )
end

"""
    plot_emw_currents()

Plot dq current references and measured/observed dq currents.
"""
function plot_emw_currents()
    emw_has_data() || return nothing
    _emw_plot(
        "FOC dq currents",
        "Time [s]",
        "Current [A]",
        "isd_ref" => emw_series(:isd_ref),
        "isd" => emw_series(:isd),
        "isq_ref" => emw_series(:isq_ref),
        "isq" => emw_series(:isq),
    )
end

"""
    plot_emw_current_errors()

Plot dq current tracking errors.
"""
function plot_emw_current_errors()
    emw_has_data() || return nothing
    _emw_plot(
        "FOC dq current errors",
        "Time [s]",
        "Current error [A]",
        "isd_error" => emw_series(:isd_error),
        "isq_error" => emw_series(:isq_error),
    )
end

"""
    plot_emw_alphabeta_currents()

Plot alpha-beta stator and rotor currents from the electrical plant.
"""
function plot_emw_alphabeta_currents()
    emw_has_data() || return nothing
    _emw_plot(
        "Electrical plant alpha-beta currents",
        "Time [s]",
        "Current [A]",
        "isα" => emw_series(:isα),
        "isβ" => emw_series(:isβ),
        "irα" => emw_series(:irα),
        "irβ" => emw_series(:irβ),
    )
end

"""
    plot_emw_voltages()

Plot dq voltage commands from the FOC current controller.
"""
function plot_emw_voltages()
    emw_has_data() || return nothing
    _emw_plot(
        "FOC dq voltages",
        "Time [s]",
        "Voltage [V]",
        "vsd" => emw_series(:vsd),
        "vsq" => emw_series(:vsq),
    )
end

"""
    plot_emw_power()

Plot electrical and mechanical power from the electric machine wrapper.
"""
function plot_emw_power()
    emw_has_data() || return nothing
    _emw_plot(
        "Electric machine power",
        "Time [s]",
        "Power [W]",
        "Pelec" => emw_series(:Pelec),
        "Pmech" => emw_series(:Pmech),
    )
end

"""
    plot_emw_saturation()

Plot FOC/current-controller saturation flag.
"""
function plot_emw_saturation()
    emw_has_data() || return nothing
    _emw_plot(
        "FOC saturation flag",
        "Time [s]",
        "Saturation [-]",
        "saturated" => emw_series(:saturated),
    )
end

"""
    print_emw_summary()

Print quick scalar metrics from the electric-machine debug log.
"""
function print_emw_summary()
    emw_has_data() || return nothing

    v_err = emw_series(:v_ro_error)
    wm_err = emw_series(:wm_error)
    Te = emw_series(:Te)
    Te_ref = emw_series(:Te_ref)
    Tload = emw_series(:Tload)
    isd = emw_series(:isd)
    isq = emw_series(:isq)
    isd_err = emw_series(:isd_error)
    isq_err = emw_series(:isq_error)
    vsd = emw_series(:vsd)
    vsq = emw_series(:vsq)
    sat = emw_series(:saturated)
    nsub = emw_series(:n_drive_substeps)

    println("ElectricMachineWinch debug summary")
    println("----------------------------------")
    println("Samples:                        ", length(EMW_DEBUG_LOG))
    println("Outer dt [s]:                   ", first(emw_series(:outer_dt)))
    println("Drive Ts [s]:                   ", first(emw_series(:drive_Ts)))
    println("Drive substeps / outer step:    ", first(nsub))
    println("Speed RMSE [m/s]:               ", sqrt(mean(abs2, v_err)))
    println("Max |speed error| [m/s]:        ", maximum(abs.(v_err)))
    println("Machine speed RMSE [rad/s]:     ", sqrt(mean(abs2, wm_err)))
    println("Max |Te| [Nm]:                  ", maximum(abs.(Te)))
    println("Max |Te_ref| [Nm]:              ", maximum(abs.(Te_ref)))
    println("Max |Tload| [Nm]:               ", maximum(abs.(Tload)))
    println("Max |isd| [A]:                  ", maximum(abs.(isd)))
    println("Max |isq| [A]:                  ", maximum(abs.(isq)))
    println("Max |isd error| [A]:            ", maximum(abs.(isd_err)))
    println("Max |isq error| [A]:            ", maximum(abs.(isq_err)))
    println("Max |vsd| [V]:                  ", maximum(abs.(vsd)))
    println("Max |vsq| [V]:                  ", maximum(abs.(vsq)))
    if :Pmech_tether in keys(first(EMW_DEBUG_LOG))
        println("Mean Pmech tether [W]:          ", mean(emw_series(:Pmech_tether)))
        println("Mean Pelec abc [W]:             ", mean(emw_series(:Pelec_abc)))
    end
    println("Saturation fraction [-]:        ", mean(sat))
    return nothing
end

"""
    save_emw_debug_csv(filename=joinpath(OUTPUT_DIR, "emw_debug.csv"))

Save the electric-machine debug log as CSV-compatible text.
"""
function save_emw_debug_csv(filename::AbstractString = joinpath(OUTPUT_DIR, "emw_debug.csv"))
    emw_has_data() || return nothing

    names_ = collect(keys(first(EMW_DEBUG_LOG)))
    header = String.(names_)
    data = Matrix{Any}(undef, length(EMW_DEBUG_LOG) + 1, length(names_))

    data[1, :] .= header

    for (r, row) in enumerate(EMW_DEBUG_LOG)
        for (c, name) in enumerate(names_)
            data[r + 1, c] = getfield(row, name)
        end
    end

    writedlm(filename, data, ',')
    println("Saved ElectricMachineWinch debug CSV to: ", abspath(filename))
    return filename
end



function clear_emw_debug_log!()
    empty!(EMW_DEBUG_LOG)
    println("ElectricMachineWinch debug log cleared.")
    return nothing
end

if __PRECOMPILE__
    app.max_time = 30
    app.next_max_time = 30
    play(false)
else
    app.viewer.menu_rel_tol.i_selected[]=2
    app.viewer.menu_rel_tol.i_selected[]=DEFAULT_TOLERANCE
    play(true)
end
if __PRECOMPILE__ && !Sys.isapple()
    do_menu(app.viewer.menu.selection[])
    sleep(0.1)   
end
stop_()
if !__PRECOMPILE__
    KiteViewers.GLMakie.closeall()   
end

GC.enable(true)
nothing

# GC disabled, Ryzen 7950X, 4x realtime, GMRES
# abs_tol: 0.0003, rel_tol: 0.0005
# Missed the deadline for 0.04 %. Max time: 172.1 ms
#     Mean    time per timestep: 3.5468899328260868 ms
#     Maximum time per timestep: 13.760848 ms
#     Maximum for t>12s        : 13.760848 ms
# Maximal memory usage: 27.0 GB

# GC disabled, Ryzen 7950X, 4x realtime, DFBDF solver
# abs_tol: 0.0003, rel_tol: 0.0005
# Missed the deadline for 0.0 %. Max time: 25.0 ms
#     Mean    time per timestep: 0.7769367125 ms
#     Maximum time per timestep: 8.064576 ms
#     Maximum for t>12s        : 7.994796 ms
# Maximal memory usage: 11.4 GB

# GC disabled, Ryzen 7950X, 4x realtime, DImplicitEuler solver
# abs_tol: 0.0003, rel_tol: 0.0005
# Missed the deadline for 0.02 %. Max time: 80.2 ms
#     Mean    time per timestep: 0.9781242155434784 ms
#     Maximum time per timestep: 17.54421 ms
#     Maximum for t>12s        : 16.454081 ms
# Maximal memory usage: 12.7 GB
