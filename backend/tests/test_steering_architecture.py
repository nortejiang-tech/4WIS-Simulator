"""Tests — the architecture registry.

Most of these assert *refusals*. An architecture layer that answers every
question for every configuration is decoration; the value is entirely in
saying no to the questions that have no meaning, before a number that means
nothing reaches a report.
"""

from __future__ import annotations

import pytest

from sim4wis.steering import architecture as arch


class TestRegistry:
    def test_covers_the_spine_the_brief_asked_for(self):
        # EPS -> SBW -> RWS -> coordinated -> 4WIS
        assert {"c_eps", "p_eps", "dp_eps", "r_eps", "sbw",
                "eps_rws", "sbw_rws", "4wis"} <= set(arch.BY_ID)

    def test_the_default_is_a_mechanical_assisted_front_axle(self):
        a = arch.get(arch.DEFAULT_ID)
        assert a.front_path == "mechanical" and a.can(arch.CAP_TORQUE_SENSOR)

    def test_unknown_ids_name_the_alternatives(self):
        with pytest.raises(arch.ArchitectureError, match="available"):
            arch.get("hydraulic")

    def test_every_entry_round_trips_as_data(self):
        for d in arch.describe_all():
            assert d["id"] and d["motor_gear_ratio"] > 0
            assert d["sensors"] and d["actuators"] and d["provides"]


class TestTopology:
    def test_only_mechanical_front_axles_have_a_torque_sensor(self):
        for a in arch.REGISTRY:
            assert a.can(arch.CAP_TORQUE_SENSOR) == (a.front_path == "mechanical")

    def test_only_by_wire_synthesises_road_feel(self):
        for a in arch.REGISTRY:
            assert a.can(arch.CAP_ROAD_FEEL) == (a.front_path == "by_wire")

    def test_assist_calibration_exists_exactly_where_assist_does(self):
        for a in arch.REGISTRY:
            assert a.can(arch.CAP_ASSIST_CAL) == (a.assist_at != "none")

    def test_rear_steer_tracks_the_rear_axle(self):
        for a in arch.REGISTRY:
            assert a.can(arch.CAP_REAR_STEER) == (a.rear_axle != "none")

    def test_per_wheel_steering_is_4wis_only(self):
        assert [a.id for a in arch.REGISTRY if a.can(arch.CAP_PER_WHEEL)] == ["4wis"]

    def test_every_architecture_can_report_hand_torque(self):
        # Either it has a mechanical path or it has a feedback actuator. An
        # architecture where the driver feels nothing is not shippable.
        assert all(a.can(arch.CAP_HAND_TORQUE) for a in arch.REGISTRY)


class TestGearRatio:
    def test_the_ratio_belongs_to_the_architecture_not_the_vehicle(self):
        # This is what the legacy single global value could not express: a
        # column worm drive and a rack ball screw differ by 3x.
        assert arch.get("c_eps").motor_gear_ratio < arch.get("r_eps").motor_gear_ratio / 2

    def test_rack_assist_matches_the_derivation(self):
        # 2*pi*i_belt/lead * r_pinion = 2*pi*2.5/0.005 * 0.020
        assert arch.get("r_eps").motor_gear_ratio == pytest.approx(63.0, rel=0.02)

    def test_assist_point_orders_the_ratios(self):
        eps = [a for a in arch.REGISTRY if a.assist_at != "none"]
        order = {"column": 0, "pinion": 1, "dual_pinion": 2, "rack": 3}
        ratios = sorted(eps, key=lambda a: order[a.assist_at])
        assert [a.motor_gear_ratio for a in ratios] == sorted(
            a.motor_gear_ratio for a in ratios
        )


class TestCapabilityGuard:
    def test_a_column_eps_cannot_be_asked_about_rear_phase(self):
        problems = arch.check_capabilities("c_eps", [arch.CAP_REAR_STEER])
        assert len(problems) == 1
        assert "eps_rws" in problems[0]["use_instead"]

    def test_a_satisfiable_request_reports_nothing(self):
        assert arch.check_capabilities("eps_rws", [
            arch.CAP_REAR_STEER, arch.CAP_HAND_TORQUE, arch.CAP_ASSIST_CAL,
        ]) == []

    def test_by_wire_has_no_assist_curve_to_calibrate(self):
        problems = arch.check_capabilities("sbw", [arch.CAP_ASSIST_CAL])
        assert problems and "r_eps" in problems[0]["use_instead"]


class TestFaultApplicability:
    def test_loss_of_assist_is_meaningless_on_a_by_wire_front_axle(self):
        # Accepting it quietly would put a result in a safety report that
        # describes nothing that can happen.
        assert arch.applicable_faults("sbw", ["assist_loss"]) == ["assist_loss"]
        assert arch.applicable_faults("r_eps", ["assist_loss"]) == []

    def test_rear_faults_need_a_rear_axle(self):
        assert arch.applicable_faults("r_eps", ["rear_stuck"]) == ["rear_stuck"]
        assert arch.applicable_faults("eps_rws", ["rear_stuck"]) == []

    def test_single_wheel_failure_belongs_to_4wis(self):
        assert arch.applicable_faults("4wis", ["single_wheel_failure"]) == []
        assert arch.applicable_faults("eps_rws", ["single_wheel_failure"])

    def test_by_wire_carries_the_comms_and_redundancy_modes(self):
        for aid in ("sbw", "sbw_rws", "4wis"):
            assert arch.applicable_faults(aid, ["comms_loss", "redundancy_switch"]) == []
