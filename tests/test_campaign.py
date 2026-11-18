from scada.campaign import campaign, run_experiment


def test_experiment_yield_peaks_near_optimum():
    assert run_experiment(130.0, 6.0) > run_experiment(60.0, 9.5) + 0.3


def test_campaign_improves_over_random_start():
    hist = campaign(n_init=4, n_iter=8, seed=0)
    best_random = max(h["yield"] for h in hist if h["phase"] == "random")
    best_bayes = max(h["yield"] for h in hist if h["phase"] == "bayes")
    assert best_bayes > best_random
    assert best_bayes > 0.85  # simulated optimum is ~1.0
