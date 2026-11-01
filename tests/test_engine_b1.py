"""Calculation model of B1: line rules, scenario builder."""
import pytest

from factory_b import Sim, make_scenario
from factory_b.defects import is_defective
from factory_b.engine import NGWAIT

H = 1800.0


def test_only_b1_is_available():
    with pytest.raises(ValueError):
        make_scenario("B2", "ii")


def test_full_bin_blocks_last_stage_until_shipped():
    s = Sim(make_scenario("B1", "ii"), 0)
    s.advance_to(300.0)  # no robot: M4 produces until the bin (2) is full, then blocks
    assert s.out_bin <= 2
    s.advance_to(1200.0)
    assert s.out_bin == 2 and s.shipped == 0


def test_ng_bin_blocks_stage_until_scrap_is_collected():
    sc = make_scenario("B1", "ii").with_condition(1.0, 10.0, False)  # every piece defective
    s = Sim(sc, 0)
    s.advance_to(200.0)
    assert s.ng_bin[0] == 1 and s.ph[0] == NGWAIT  # second defect of M1 waits for the bin
    assert [x for x in s.candidates() if x.job == ("COL", 0)]
    s.now = 200.0
    s.execute(("COL", 0), H)
    assert s.ph[0] != NGWAIT and s.ng_bin[0] == 1


def test_ct_scale_scales_cycle_times_only():
    base = make_scenario("B1", "ii")
    sc = make_scenario("B1", "ii", ct_scale=0.5)
    assert sc.ct == tuple(c * 0.5 for c in base.ct)
    assert sc.bs == base.bs and sc.cap == base.cap and sc.init_std == base.init_std
    assert make_scenario("B1", "ii", ct_scale=1.0).ct == (64.0, 72.0, 86.0, 76.0)


def test_defect_series_depends_only_on_seed_machine_cycle():
    a = [is_defective(5, "M1", n, 10.0) for n in range(1, 200)]
    assert a == [is_defective(5, "M1", n, 10.0) for n in range(1, 200)]
    assert 5 <= sum(a) <= 40 and not is_defective(5, "M1", 1, 0.0) and is_defective(5, "M1", 1, 100.0)
