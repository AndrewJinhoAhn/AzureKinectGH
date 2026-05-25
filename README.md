# Azure Kinect for Grasshopper

Real-time Azure Kinect DK integration for Grasshopper. Live body tracking,
point cloud streaming with floor calibration, and quad mesh generation —
all from a single shared capture stream.

## Features

- **Kinect Device** — Activate camera, configure FPS and depth mode,
  IMU-based automatic floor calibration.
- **Body Tracker** — Skeleton tracking with persistent body IDs and
  per-joint confidence threshold filtering. DirectML-accelerated, runs
  on any DirectX 12 GPU (no CUDA installation required).
- **Point Cloud** — Structured point grid, downsample stride for
  performance, floor-corrected coordinates.
- **Mesh from Grid** — Optional quad mesh from point grid, max-edge
  culling for clean surface separation.

## Architecture

All components share a single capture stream via a background thread.
Body Tracker and Point Cloud see the same frame without competing for
captures. GH solver runs immediately without waiting for K4A.

## Requirements

| Component | Requirement                                                         |
| --------- | ------------------------------------------------------------------- |
| OS        | Windows 10 (1903+) or Windows 11, 64-bit                            |
| App       | Rhino 8                                                             |
| Hardware  | Azure Kinect DK                                                     |
| GPU       | Any DirectX 12 GPU (NVIDIA / AMD / Intel iGPU)                      |
| Driver    | Azure Kinect USB driver (usually auto-installed via Windows Update) |

The Azure Kinect Sensor SDK and Body Tracking SDK runtimes are bundled
with this plugin — you do **not** need to install them separately.

## Installation

### Via Rhino Package Manager (recommended)

In Rhino, type `_PackageManager` → search "Azure Kinect" → Install.

### Manual

1. Download the latest `.yak` from the [Releases](../../releases) page.
2. In Rhino, type `_PackageManager` → File → Install from file → select
   the `.yak`.

## Usage

Basic workflow:

1. Drop **Kinect Device** on the canvas, connect a `Boolean Toggle` to
   the `Active` input, set it to `true`. The device starts.
2. Click **Recalibrate** once to compute the floor transform from IMU
   (camera should be in its final mounted position).
3. Drop **Body Tracker** and/or **Point Cloud**, connect to Device's
   `D
