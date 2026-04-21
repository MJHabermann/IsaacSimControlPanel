import omni.timeline
import omni.kit.app


_STATE = {
    "variable_name": "restart",
    "did_log_first_compute": False,
    "pending_play_sub": None,
    "wait_frames": 0,
}


def _get_input(db, name, default):
    try:
        value = getattr(db.inputs, name)
        return default if value is None else value
    except Exception:
        return default


def _deferred_play(event, db_ref):
    # Wait a couple update frames after stop() before play() so the
    # timeline has actually processed the stop.
    _STATE["wait_frames"] -= 1
    if _STATE["wait_frames"] > 0:
        return

    try:
        omni.timeline.get_timeline_interface().play()
    except Exception:
        pass

    if _STATE["pending_play_sub"] is not None:
        _STATE["pending_play_sub"] = None  # unsubscribe by dropping reference


def setup(db):
    _STATE["variable_name"] = str(_get_input(db, "variableName", "restart"))
    _STATE["did_log_first_compute"] = False
    _STATE["pending_play_sub"] = None
    _STATE["wait_frames"] = 0
    db.log_warning("[restart_sim] setup() executed.")


def compute(db):
    if not _STATE["did_log_first_compute"]:
        db.log_warning("[restart_sim] compute() is running.")
        _STATE["did_log_first_compute"] = True

    if not bool(_get_input(db, "restart", False)):
        return True

    try:
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()

        # Schedule play() to run on a future update tick
        _STATE["wait_frames"] = 3
        app = omni.kit.app.get_app()
        _STATE["pending_play_sub"] = (
            app.get_update_event_stream().create_subscription_to_pop(
                lambda e: _deferred_play(e, db),
                name="restart_sim_deferred_play",
            )
        )
        db.log_warning("[restart_sim] Stop issued, play scheduled.")
    except Exception as e:
        db.log_warning(f"[restart_sim] Timeline restart failed: {e}")
        return True

    # Flip the graph variable back to False so the trigger is one-shot.
    try:
        graph = db.node.get_graph()
        var = graph.find_variable(_STATE["variable_name"])
        if var is None:
            available = [v.name for v in graph.get_variables()]
            db.log_warning(
                f"[restart_sim] Variable '{_STATE['variable_name']}' not found. "
                f"Available: {available}"
            )
            return True

        var.set(graph.get_context(), False)
    except Exception as e:
        db.log_warning(f"[restart_sim] Could not reset variable: {e}")

    return True