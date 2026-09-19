# Pose model

`rtmpose-m-wholebody.onnx` is the unmodified OpenMMLab RTMPose-M
COCO-WholeBody export (256×192 input, 133 SimCC joints). The application maps
body and foot joints to BODY_25 and derives neck and mid-hip from paired joints.
Model weights are ignored by Git.

- [Official model catalog](https://github.com/open-mmlab/mmpose/tree/main/projects/rtmpose)
- [Download ZIP](https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/rtmpose-m_simcc-ucoco_dw-ucoco_270e-256x192-c8b76419_20230728.zip)
- Extract `end2end.onnx` from the ZIP to `resources/models/rtmpose-m-wholebody.onnx`.
- SHA-256: `94ca58fa2d6c4530b6957ac9548084ebc2fa27ed71e4e01f0b73844306ed01a6`
- The accompanying `rtmpose-m-wholebody.pipeline.json` is the upstream preprocessing/decoder specification.

This is a top-down, single-person pose model. The current app supplies the
whole camera image as its person region; keep one person clearly visible.
It does not perform person detection or single-camera 3D reconstruction.
Use two or more calibrated views for 3D skeletons.

IP capture continuously reads into a single latest-frame slot, including while
capture is stopped. Unconsumed frames are discarded. Delivery to the GUI is
also coalesced so slow inference cannot accumulate stale frame events. Opening
a stream times out after 5 seconds, and a blocked read after 1 second. A failed
read marks the camera disconnected; reopen it through camera management.
The actual latency also depends on the camera encoder, network, and decoder.
