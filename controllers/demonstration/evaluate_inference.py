"""
Evaluation script for ACT inference system.
Runs multiple trials and tracks success rates for the dishwasher closing task.
"""

import os
import sys
import time
from run_inference_temporalAgg import ACTInference, INFERENCE_STEP


def run_evaluation_trials(num_trials=10):
    """Run multiple trials and track success rates"""
    # Paths
    ckpt_path = "policy_best.ckpt"
    dataset_stats_path = "dataset_stats2.pkl"
    
    # Check if checkpoint exists
    if not os.path.exists(ckpt_path):
        print(f"Error: Checkpoint file not found: {ckpt_path}")
        return None
    
    print(f"Starting {num_trials}-trial evaluation...")
    print(f"Each trial runs for maximum {INFERENCE_STEP} steps")
    
    # Track results across trials
    trial_results = []
    successful_trials = 0
    
    for trial in range(num_trials):
        print(f"\n{'='*50}")
        print(f"TRIAL {trial + 1}/{num_trials}")
        print(f"{'='*50}")
        
        # Initialize fresh inference system for each trial
        try:
            inference = ACTInference(ckpt_path, dataset_stats_path)
            
            # Run single trial
            results = inference.run_inference(max_timesteps=INFERENCE_STEP)
            trial_results.append(results)
            
            # Track success
            if results['success']:
                successful_trials += 1
                print(f"✅ TRIAL {trial + 1} SUCCESSFUL")
                print(f"   Completed in {results['duration']:.1f}s at step {results['completion_step']}")
                print(f"   Completion rate: {results['completion_rate']:.1%}")
            else:
                print(f"❌ TRIAL {trial + 1} FAILED")
                print(f"   Ran for {results['duration']:.1f}s, {results['total_steps']} steps")
            
            # Brief pause between trials
            if trial < num_trials - 1:
                print("Preparing for next trial...")
                time.sleep(2)
                
        except Exception as e:
            print(f"❌ TRIAL {trial + 1} ERROR: {e}")
            # Create a failed result entry
            trial_results.append({
                'success': False,
                'completion_step': None,
                'duration': 0.0,
                'total_steps': 0,
                'final_dishwasher_state': None,
                'completion_rate': None,
                'error': str(e)
            })
    
    # Calculate final statistics
    success_rate = successful_trials / num_trials
    avg_duration = sum(r['duration'] for r in trial_results) / num_trials
    avg_steps = sum(r['total_steps'] for r in trial_results) / num_trials
    
    successful_results = [r for r in trial_results if r['success']]
    if successful_results:
        avg_completion_step = sum(r['completion_step'] for r in successful_results) / len(successful_results)
        avg_successful_duration = sum(r['duration'] for r in successful_results) / len(successful_results)
        min_completion_step = min(r['completion_step'] for r in successful_results)
        max_completion_step = max(r['completion_step'] for r in successful_results)
    else:
        avg_completion_step = None
        avg_successful_duration = None
        min_completion_step = None
        max_completion_step = None
    
    # Print final evaluation summary
    print(f"\n{'='*60}")
    print(f"EVALUATION COMPLETE - {num_trials} TRIALS")
    print(f"{'='*60}")
    print(f"🎯 SUCCESS RATE: {successful_trials}/{num_trials} = {success_rate:.1%}")
    print(f"⏱️  AVERAGE DURATION: {avg_duration:.1f}s")
    print(f"📊 AVERAGE STEPS: {avg_steps:.0f}")
    
    if successful_results:
        print(f"\n✅ SUCCESSFUL TRIALS ANALYSIS:")
        print(f"   • Count: {len(successful_results)}")
        print(f"   • Average completion step: {avg_completion_step:.0f}")
        print(f"   • Average successful duration: {avg_successful_duration:.1f}s")
        print(f"   • Fastest completion: step {min_completion_step}")
        print(f"   • Slowest completion: step {max_completion_step}")
    
    failed_results = [r for r in trial_results if not r['success']]
    if failed_results:
        print(f"\n❌ FAILED TRIALS ANALYSIS:")
        print(f"   • Count: {len(failed_results)}")
        avg_failed_duration = sum(r['duration'] for r in failed_results) / len(failed_results)
        avg_failed_steps = sum(r['total_steps'] for r in failed_results) / len(failed_results)
        print(f"   • Average failed duration: {avg_failed_duration:.1f}s")
        print(f"   • Average steps before failure: {avg_failed_steps:.0f}")
    
    print(f"{'='*60}")
    
    return {
        'success_rate': success_rate,
        'successful_trials': successful_trials,
        'total_trials': num_trials,
        'trial_results': trial_results,
        'avg_duration': avg_duration,
        'avg_steps': avg_steps,
        'avg_completion_step': avg_completion_step,
        'avg_successful_duration': avg_successful_duration,
        'min_completion_step': min_completion_step,
        'max_completion_step': max_completion_step
    }


def print_detailed_results(evaluation_results):
    """Print detailed trial-by-trial results"""
    print(f"\n{'='*60}")
    print("DETAILED TRIAL RESULTS")
    print(f"{'='*60}")
    
    for i, result in enumerate(evaluation_results['trial_results']):
        trial_num = i + 1
        status = "✅ SUCCESS" if result['success'] else "❌ FAILED"
        
        print(f"Trial {trial_num:2d}: {status}")
        print(f"         Duration: {result['duration']:6.1f}s")
        print(f"         Steps: {result['total_steps']:4d}")
        
        if result['success']:
            print(f"         Completed at step: {result['completion_step']}")
            print(f"         Completion rate: {result['completion_rate']:.1%}")
        elif 'error' in result:
            print(f"         Error: {result['error']}")
        
        print()


def main():
    """Main evaluation function"""
    print("ACT Inference Evaluation System")
    print("===============================")
    
    # Run evaluation with 10 trials (default)
    evaluation_results = run_evaluation_trials(num_trials=10)
    
    if evaluation_results is None:
        print("Evaluation failed - checkpoint file not found")
        return
    
    # Print detailed results
    print_detailed_results(evaluation_results)
    
    # Save results to file
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    results_file = f"evaluation_results_{timestamp}.txt"
    
    with open(results_file, 'w') as f:
        f.write("ACT Inference Evaluation Results\n")
        f.write("=" * 40 + "\n")
        f.write(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Success Rate: {evaluation_results['success_rate']:.1%}\n")
        f.write(f"Successful Trials: {evaluation_results['successful_trials']}/{evaluation_results['total_trials']}\n")
        f.write(f"Average Duration: {evaluation_results['avg_duration']:.1f}s\n")
        f.write(f"Average Steps: {evaluation_results['avg_steps']:.0f}\n")
        
        if evaluation_results['avg_completion_step']:
            f.write(f"Average Completion Step: {evaluation_results['avg_completion_step']:.0f}\n")
            f.write(f"Average Successful Duration: {evaluation_results['avg_successful_duration']:.1f}s\n")
        
        f.write("\nDetailed Results:\n")
        for i, result in enumerate(evaluation_results['trial_results']):
            f.write(f"Trial {i+1}: {'SUCCESS' if result['success'] else 'FAILED'} - ")
            f.write(f"{result['duration']:.1f}s, {result['total_steps']} steps")
            if result['success']:
                f.write(f", completed at step {result['completion_step']}")
            f.write("\n")
    
    print(f"Results saved to: {results_file}")


if __name__ == "__main__":
    main()