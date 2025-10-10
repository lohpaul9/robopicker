#!/usr/bin/env python
"""
Test script to verify episode metadata storage works correctly.

This creates a minimal dataset, saves an episode with custom metadata,
and verifies it can be loaded back.
"""

import tempfile
from pathlib import Path
import shutil
import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset

def test_episode_metadata():
    """Test that custom episode metadata is saved and loaded correctly."""

    # Create a temporary directory for the test dataset
    with tempfile.TemporaryDirectory() as tmpdir:
        repo_id = "test/metadata"
        root = Path(tmpdir) / repo_id

        # Define minimal features
        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (2,),
                "names": ["x", "y"]
            },
            "action": {
                "dtype": "float32",
                "shape": (2,),
                "names": ["vx", "vy"]
            }
        }

        # Create dataset
        print("Creating test dataset...")
        dataset = LeRobotDataset.create(
            repo_id=repo_id,
            fps=30,
            features=features,
            root=root,
            robot_type="test_robot",
            use_videos=False
        )

        # Add some frames to the episode
        print("Adding frames to episode...")
        for i in range(5):
            frame = {
                "observation.state": np.array([i * 0.1, i * 0.2], dtype=np.float32),
                "action": np.array([0.1, 0.2], dtype=np.float32),
                "task": "test task"
            }
            dataset.add_frame(frame)

        # Create custom metadata
        episode_metadata = {
            "block/initial_x": 0.123,
            "block/initial_y": 0.456,
            "block/initial_z": 0.012,
            "test/custom_field": 42.0,
            "test/string_field": "hello"
        }

        print(f"Saving episode with metadata: {episode_metadata}")
        dataset.save_episode(episode_metadata=episode_metadata)

        # Reload the dataset and check metadata
        print("\nReloading dataset to verify metadata...")
        dataset2 = LeRobotDataset(repo_id=repo_id, root=root)

        # Access episode metadata
        episodes = dataset2.meta.episodes
        print(f"\nTotal episodes: {len(episodes)}")

        if len(episodes) > 0:
            episode_0 = episodes[0]
            print(f"\nEpisode 0 columns: {episodes.column_names}")
            print(f"\nEpisode 0 data:")
            for key in episodes.column_names:
                print(f"  {key}: {episode_0[key]}")

            # Check if our custom metadata is there
            print("\n✓ Checking custom metadata:")
            for key in episode_metadata.keys():
                if key in episodes.column_names:
                    actual_value = episode_0[key]
                    expected_value = episode_metadata[key]

                    # Handle numeric comparison with tolerance
                    if isinstance(expected_value, (int, float)):
                        if abs(actual_value - expected_value) < 1e-6:
                            print(f"  ✓ {key}: {actual_value} (expected {expected_value})")
                        else:
                            print(f"  ✗ {key}: {actual_value} (expected {expected_value}) - MISMATCH!")
                    else:
                        if actual_value == expected_value:
                            print(f"  ✓ {key}: {actual_value}")
                        else:
                            print(f"  ✗ {key}: {actual_value} (expected {expected_value}) - MISMATCH!")
                else:
                    print(f"  ✗ {key}: MISSING from episodes!")
        else:
            print("ERROR: No episodes found in dataset!")

        print("\n" + "="*60)
        print("Test completed!")
        print("="*60)

if __name__ == "__main__":
    test_episode_metadata()
