#!/usr/bin/env python3
"""Python reference for the two-stage retrieval, mirroring
Referee/util/recognition/refereer.py::extract_multi_session_refereepp_loop_pair.

Emits the same lines as tools/validate_retrieval so the two can be diffed.
"""
import sys
import numpy as np
from scipy.spatial import cKDTree


def huber_loss(a, b, delta=200000.0):        # verbatim from the reference
    diff = a - b
    abs_diff = np.abs(diff)
    delta = np.mean(abs_diff)
    is_small_error = abs_diff <= 0
    small_error_loss = 0.5 * np.square(diff)
    large_error_loss = abs_diff
    return np.mean(np.where(is_small_error, small_error_loss, large_error_loss))


def main():
    npy_dir, n = sys.argv[1], int(sys.argv[2])
    coarse = int(sys.argv[3]) if len(sys.argv) > 3 else 20
    top_k = int(sys.argv[4]) if len(sys.argv) > 4 else 10

    desc = np.array([np.load(f"{npy_dir}/{i:06d}.npy").ravel() for i in range(n)])
    split = n // 2
    database, queries = desc[:split], desc[split:]

    tree = cKDTree(database[:, :coarse])
    for q, query in enumerate(queries):
        k = min(top_k, len(database))
        _, cand = tree.query(query[:coarse], k=k)
        cand = np.atleast_1d(cand)

        inverse_weights = 1 / np.arange(1, len(query) + 1)
        scored = [(int(c), huber_loss(query * inverse_weights,
                                      database[c] * inverse_weights, delta=50.0))
                  for c in cand]
        scored.sort(key=lambda x: x[1])
        print(str(split + q) + "".join(f" {c}:{d:.6f}" for c, d in scored[:top_k]))


if __name__ == "__main__":
    main()
