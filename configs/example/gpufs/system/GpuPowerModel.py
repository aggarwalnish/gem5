from m5.objects import MathExprPowerModel, PowerModel, SubSystem
from m5.SimObject import isSimObjectVector


def _stats_path(root, target):
    """Resolve the stats hierarchy path for a SimObject.

    The stats tree is built from each object's membership in its parent's
    ``_children`` map, not from ``obj.path()``. Those can differ when code
    manually calls ``set_parent()`` without adding a corresponding child.
    """
    root_path = root.path().replace("<orphan System>", "system")

    def visit(prefix, obj):
        if obj is target:
            return prefix

        for name, child in sorted(obj._children.items()):
            if isSimObjectVector(child):
                if len(child) == 1:
                    entries = [(name, child[0])]
                else:
                    entries = [(f"{name}{idx}", elem)
                               for idx, elem in enumerate(child)]
            else:
                entries = [(name, child)]

            for child_name, child_obj in entries:
                child_path = f"{prefix}.{child_name}"
                resolved = visit(child_path, child_obj)
                if resolved:
                    return resolved

        return None

    resolved = visit(root_path, root)
    if not resolved:
        raise RuntimeError(f"Unable to resolve stats path for {target.path()}")
    return resolved

# --- Generic Power-Off Model ---
class PowerOff(MathExprPowerModel):
    """
    Model for when a component is clock gated or powered down.
    Assumes 0 dynamic power and negligible static power for this example.
    """

    dyn = "0"
    st = "0" #TODO : Realistically, Static power would be non zero for clk gate

# --- Shader-Specific Models ---
class ShaderPowerOn(MathExprPowerModel):
    def __init__(self, shader_stats_path, **kwargs):
        super().__init__(**kwargs)
        clean_path = shader_stats_path
        self.dyn = "voltage * (10.0 * {}.shaderActiveTicks / simSeconds)".format(
            clean_path
        )
        with open("/tmp/cu_power_expr.txt", "a") as f:
            f.write(
                "Shader Power: stats_path="
                f"{clean_path}, dyn={self.dyn}\n"
            )
        self.st = "4 * temp"

class GpuPowerModel(PowerModel):
    def __init__(self, shader_path, subsystem, **kwargs):
        super().__init__(**kwargs)
        self.subsystem = subsystem
        self.pm = [
            ShaderPowerOn(shader_path),
            PowerOff(),
            PowerOff(),
            PowerOff(),
        ]

# --- Compute Unit-Specific Models ---
class ComputeUnitPowerOn(MathExprPowerModel):
    def __init__(self, cu_stats_path, **kwargs):
        super().__init__(**kwargs)
        clean_path = cu_stats_path
        # Power model based on voltage squared, CRISP-window IPC, and
        # clock period.
        # This captures both voltage and frequency effects on dynamic power
        # Using parentheses for clarity and proper operator precedence
        self.dyn = "(voltage * voltage) * ({}.windowIpc / clock_period) * 1000000".format(
            clean_path
        )
        with open("/tmp/cu_power_expr.txt", "a") as f:
            f.write(
                f"CU Power: stats_path={clean_path}, dyn={self.dyn}\n"
            )
        self.st = "0.5 * temp"

class ComputeUnitPowerModel(PowerModel):
    def __init__(self, cu_path, subsystem, **kwargs):
        super().__init__(**kwargs)
        self.subsystem = subsystem
        self.pm = [
            ComputeUnitPowerOn(cu_path),
            PowerOff(),
            PowerOff(),
            PowerOff(),
        ]

# --- Attachment Helpers ---
def _attach_power_model(system, component, model_class, *model_args):
    """
    Generic helper to attach a power model to a component.
    """
    # Each component needs its own SubSystem for the power model to work.
    # We create it dynamically and attach it to the system to ensure it exists.
    subsystem_name = f"_{component.path().replace('.', '_')}_thermal_domain"
    thermal_domain = SubSystem()
    setattr(system, subsystem_name, thermal_domain)

    component.power_state.default_state = "ON"
    component.power_model = model_class(*model_args, subsystem=thermal_domain)
    print(f"DEBUG: Power Model attached to {component.path()}")


def attach_gpu_power(system, shader):
    """
    Attaches the power model to the entire GPU Shader.
    """
    _attach_power_model(system, shader, GpuPowerModel,
                        _stats_path(system, shader))


def attach_cu_power(system, cu):
    """
    Attaches the power model to a single Compute Unit.
    """
    _attach_power_model(system, cu, ComputeUnitPowerModel,
                        _stats_path(system, cu))
