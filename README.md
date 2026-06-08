# Azure Kinect for Grasshopper

Real-time Azure Kinect DK integration for Grasshopper. Live body tracking, point cloud streaming with floor calibration, and quad mesh generation — all from a single shared capture stream.

> Part of the **Appendage** plugin family — hardware integrations for Rhino/Grasshopper.  
> Components appear under the **Appendage** tab → **Kinect** panel in Grasshopper.

## Features

| Component | Description |
|---|---|
| **Kinect Device** | Activate camera, configure FPS and depth mode, IMU-based automatic floor calibration. |
| **Body Tracker** | Skeleton tracking with persistent body IDs and per-joint confidence threshold filtering. DirectML-accelerated, runs on any DirectX 12 GPU. |
| **Point Cloud** | Structured point grid, downsample stride for performance, floor-corrected coordinates. |
| **Mesh from Grid** | Optional quad mesh from point grid, max-edge culling for clean surface separation. |

## Architecture

All components share a single capture stream via a background thread. Body Tracker and Point Cloud see the same frame without competing for captures. GH solver runs immediately without waiting for K4A.

## Requirements

| | Requirement |
|---|---|
| OS | Windows 10 (1903+) or Windows 11, 64-bit |
| App | Rhino 8 |
| Hardware | Azure Kinect DK |
| GPU | Any DirectX 12 GPU (NVIDIA / AMD / Intel iGPU) |

The Azure Kinect Sensor SDK and Body Tracking SDK runtimes are bundled — no separate installation needed.

## Installation

**Via Rhino Package Manager (recommended)**

In Rhino, type `_PackageManager` → search "Azure Kinect" → Install.

**Manual**

1. Download the latest `.yak` from the [Releases](../../releases) page.
2. In Rhino, type `_PackageManager` → File → Install from file → select the `.yak`.

## Usage

1. Drop **Kinect Device** on the canvas, connect a Boolean Toggle to `Active`, set to `true`.
2. Click **Recalibrate** once to compute the floor transform from IMU.
3. Drop **Body Tracker** and/or **Point Cloud**, connect to the Device output.
4. Optionally drop **Mesh from Grid** downstream of Point Cloud.
