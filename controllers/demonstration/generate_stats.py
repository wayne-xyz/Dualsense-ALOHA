#!/usr/bin/env python3
"""
Generate dataset statistics from collected data for ACT model normalization
"""

import numpy as np
import h5py
import os
import pickle
from pathlib import Path


def compute_dataset_stats(data_dir="../../data"):
    """
    Compute mean and std statistics from all episodes in data directory
    """
    data_path = Path(data_dir)
    
    if not data_path.exists():
        print(f"Data directory not found: {data_dir}")
        return None
    
    episode_files = list(data_path.glob("episode_*.hdf5"))
    
    if not episode_files:
        print(f"No episode files found in: {data_dir}")
        return None
    
    print(f"Found {len(episode_files)} episode files")
    
    all_qpos = []
    all_actions = []
    
    for episode_file in episode_files:
        print(f"Processing: {episode_file.name}")
        
        with h5py.File(episode_file, 'r') as f:
            # Load qpos and actions
            qpos = f['observations/qpos'][:]
            action = f['action'][:]
            
            all_qpos.append(qpos)
            all_actions.append(action)
    
    # Concatenate all episodes
    all_qpos = np.concatenate(all_qpos, axis=0)
    all_actions = np.concatenate(all_actions, axis=0)
    
    print(f"Total samples: {len(all_qpos)}")
    print(f"qpos shape: {all_qpos.shape}")
    print(f"action shape: {all_actions.shape}")
    
    # Compute statistics
    qpos_mean = np.mean(all_qpos, axis=0)
    qpos_std = np.std(all_qpos, axis=0)
    action_mean = np.mean(all_actions, axis=0)
    action_std = np.std(all_actions, axis=0)
    
    # Avoid division by zero
    qpos_std = np.maximum(qpos_std, 1e-6)
    action_std = np.maximum(action_std, 1e-6)
    
    stats = {
        'qpos_mean': qpos_mean,
        'qpos_std': qpos_std,
        'action_mean': action_mean,
        'action_std': action_std,
    }
    
    print("Dataset Statistics:")
    print(f"  qpos_mean: {qpos_mean}")
    print(f"  qpos_std: {qpos_std}")
    print(f"  action_mean: {action_mean}")
    print(f"  action_std: {action_std}")
    
    return stats


def main():
    """Main function"""
    # Compute stats from data directory
    stats = compute_dataset_stats("../../data")
    
    if stats is None:
        print("Failed to compute dataset statistics")
        return
    
    # Save stats
    stats_path = "dataset_stats2.pkl"
    with open(stats_path, 'wb') as f:
        pickle.dump(stats, f)
    
    print(f"\nDataset statistics saved to: {stats_path}")
    print("You can now run inference with proper normalization!")


if __name__ == "__main__":
    main()