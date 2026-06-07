class RandomReferenceSpeed:
    def __init__(
        self,
        node,
        enabled,
        scenario_id,
        loop,
        fallback_velocity_mps,
        initial_velocity_mps,
        change_times_sec,
        change_velocities_mps,
        min_velocity_mps,
        max_velocity_mps,
    ):
        self.node = node
        self.enabled = bool(enabled)
        self.scenario_id = int(scenario_id)
        self.loop = bool(loop)
        self.fallback_velocity_mps = float(fallback_velocity_mps)
        self.initial_velocity_mps = float(initial_velocity_mps)
        self.change_times_sec = [float(value) for value in change_times_sec]
        self.change_velocities_mps = [float(value) for value in change_velocities_mps]
        self.min_velocity_mps = float(min_velocity_mps)
        self.max_velocity_mps = float(max_velocity_mps)
        self.last_reference_velocity_mps = None

        if self.min_velocity_mps > self.max_velocity_mps:
            self.node.get_logger().warn(
                "Random reference min velocity is greater than max; swapping."
            )
            self.min_velocity_mps, self.max_velocity_mps = (
                self.max_velocity_mps,
                self.min_velocity_mps,
            )

        self.validate()

    def validate(self):
        if not self.enabled:
            return

        if len(self.change_times_sec) != len(self.change_velocities_mps):
            self.node.get_logger().warn(
                "Random reference change_times and change_velocities lengths differ; "
                "disabling random reference."
            )
            self.enabled = False
            return

        if not self.change_times_sec:
            self.node.get_logger().warn(
                "Random reference enabled but no changes are configured; "
                "disabling random reference."
            )
            self.enabled = False
            return

        previous_time = 0.0
        for change_time in self.change_times_sec:
            if change_time <= previous_time:
                self.node.get_logger().warn(
                    "Random reference change times must be strictly increasing; "
                    "disabling random reference."
                )
                self.enabled = False
                return
            previous_time = change_time

    def clamp(self, velocity_mps):
        return max(
            self.min_velocity_mps, min(self.max_velocity_mps, float(velocity_mps))
        )

    def get_reference_velocity(self, elapsed_sec):
        if not self.enabled:
            return self.clamp(self.fallback_velocity_mps)

        scenario_time = max(0.0, float(elapsed_sec))
        final_time = self.change_times_sec[-1]
        if self.loop and final_time > 0.0:
            scenario_time = scenario_time % final_time

        reference_velocity_mps = self.initial_velocity_mps
        for change_time, change_velocity_mps in zip(
            self.change_times_sec, self.change_velocities_mps
        ):
            if scenario_time < change_time:
                break
            reference_velocity_mps = change_velocity_mps

        reference_velocity_mps = self.clamp(reference_velocity_mps)
        self.log_if_changed(reference_velocity_mps)
        return reference_velocity_mps

    def log_if_changed(self, reference_velocity_mps):
        if (
            self.last_reference_velocity_mps is not None
            and abs(reference_velocity_mps - self.last_reference_velocity_mps) < 1.0e-9
        ):
            return

        self.last_reference_velocity_mps = reference_velocity_mps
        if self.enabled:
            self.node.get_logger().info(
                "Reference speed scenario changed: "
                f"scenario={self.scenario_id}, ref_vx={reference_velocity_mps:.3f} m/s"
            )
