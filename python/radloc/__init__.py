"""RadLoc: radar place recognition and 3-DoF registration.

This package is the research reference. It mirrors the header-only C++ core
under ``cpp/radloc`` module for module, and the two are checked against each
other and against the archived descriptors by ``tests/``. The C++ side is what
the ROS 2 nodes use.
"""

from .descriptor import (
    DescriptorParams,
    ca_cfar_1d,
    compute_descriptor,
    from_inverted,
    range_weighted,
    read_descriptor,
    write_descriptor,
)
from .polar_image import load_polar_image, to_cartesian_points, to_polar_scan
from .retrieval import FineMetric, PlaceDatabase, RetrievalParams, fine_distance

__all__ = [
    "DescriptorParams",
    "FineMetric",
    "PlaceDatabase",
    "RetrievalParams",
    "ca_cfar_1d",
    "compute_descriptor",
    "fine_distance",
    "from_inverted",
    "load_polar_image",
    "range_weighted",
    "read_descriptor",
    "to_cartesian_points",
    "to_polar_scan",
    "write_descriptor",
]
