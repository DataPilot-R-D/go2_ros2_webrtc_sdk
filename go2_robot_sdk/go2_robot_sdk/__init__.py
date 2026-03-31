import sys
import os
import logging

logger = logging.getLogger(__name__)

# Import unitree_webrtc_connect FIRST, before aiortc.
# Its __init__.py monkey-patches aioice.Connection (shared ICE credentials)
# and aiortc.rtcdtlstransport.X509_DIGEST_ALGORITHMS (SHA-256 only).
# These patches MUST be applied before aiortc is imported, because
# aiortc.rtcicetransport does `from aioice import Connection` at import
# time, caching a reference to the Connection class. If aiortc is imported
# first, its cached reference points to the unpatched class, and each ICE
# connection gets unique credentials instead of shared ones — which breaks
# the Go2's BUNDLE negotiation and causes STUN BINDING ERRORs.
try:
    import unitree_webrtc_connect  # noqa: F401 — patches aioice + aiortc
    # Also patch the already-cached reference in rtcicetransport if aiortc
    # was somehow imported before us (belt-and-suspenders)
    try:
        import aioice
        from aiortc import rtcicetransport
        rtcicetransport.Connection = aioice.Connection
    except ImportError:
        pass
except ImportError:
    logger.warning('unitree_webrtc_connect not installed — WebRTC will not work')
