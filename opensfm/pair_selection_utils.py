from opensfm import log


def processes_that_fit_in_memory(desired):
    """Amount of parallel matching process that fit in memory."""
    per_process_mem = 1.6 * 1024
    available_mem = log.memory_available()
    if available_mem is not None:
        fittable = max(1, int(available_mem / per_process_mem))
        return min(desired, fittable)
    else:
        return desired


def keep_first_word(words):
    """Keep only the first word of each feature.

    This is useful to free memory when only the first word is going to
    be used.  It copies the array so that the original words array can
    be freed.
    """
    if words is None or len(words) == 0:
        return words
    return words[:, :1].copy()