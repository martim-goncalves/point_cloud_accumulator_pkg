# Point Cloud Accumulator

## 1. Introduction
This ROS 2 package is responsible for ingesting point cloud frames and accumulating them into a fused point cloud. A chain of filters is applied to each frame before accumulation. The fused cloud is downsampled to mitigate unbounded growth. Both the intermediate, filtered frames and the accumulated cloud are published to ROS 2 topics for ingestion by other nodes or visualization. If the save timer interval is set to zero, the accumulated cloud will only be saved on shutdown.



## 2. Parameters
+ Describe each parameter and its purpose



## 3. Structure
+ Spatial indexing data structures (e.g. KD-Trees, Octrees, etc.)



## 4. Dynamic
...

### 4.1. Filter Chains
+ Callback Group for Filtering:
    * Dedicate a single-threaded group to receive raw clouds and push them into a lock-free queue.

+ Worker Thread Pool for Heavy Lifting:
    * Have one or two threads pull off that queue, run the filter chain and accumulation, then publish.
    * This way a slow VoxelGrid on a huge map won’t block your subscriber.

#### 4.1.1. Frame Processing
...

#### 4.1.2. Accumulator Processing
...



## 5. Usage
+ How to run it (i.e. snippets and such)
