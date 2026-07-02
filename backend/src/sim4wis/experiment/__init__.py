"""Experiment / batch-run subsystem (platform refactor Phase A).

An *Experiment* is the reproducible unit of work: vehicle + model + strategy +
scene + path + a sim-time maneuver + recording/KPI settings, stored as YAML.
A *Run* is one headless execution of an experiment; it leaves an artifact
(`runs/<id>/meta.json + data.csv`) that the analysis UI reads back.
"""
