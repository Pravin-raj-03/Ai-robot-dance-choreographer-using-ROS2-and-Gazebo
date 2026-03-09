import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import numpy as np
import threading
import time
import random

import tkinter as tk
from tkinter import ttk, filedialog

try:
    import pygame
except ImportError:
    pygame = None

try:
    import librosa
except ImportError:
    librosa = None


def detect_beats(audio_path):
    """Return a list of beat timestamps (seconds) from an audio file."""
    if librosa is not None:
        try:
            y, sr = librosa.load(audio_path, duration=120)
            _, beat_frames = librosa.beat.beat_track(y=y, sr=sr)
            return librosa.frames_to_time(beat_frames, sr=sr).tolist()
        except Exception as e:
            print(f'librosa beat detection failed: {e}')

    # Fallback: simple energy-based detection using scipy
    try:
        import scipy.io.wavfile as wav
        fs, data = wav.read(audio_path)
        if data.ndim > 1:
            data = data.mean(axis=1)
        step = int(fs / 50)
        envelope = np.abs(data[::step].astype(float))
        times = np.arange(len(envelope)) * (step / fs)

        window = 50
        rolling_mean = np.convolve(envelope, np.ones(window) / window, mode='same')
        peaks = np.where((envelope > rolling_mean * 1.3) & (envelope > 0.01))[0]

        min_gap = int(0.35 * 50)
        beats, last = [], -min_gap
        for idx in peaks:
            if idx - last > min_gap:
                beats.append(times[idx])
                last = idx
        if beats:
            return beats
    except Exception as e:
        print(f'Fallback beat detection failed: {e}')

    # Last resort: fixed 120 BPM
    return np.arange(0, 60, 0.5).tolist()


class DanceGuiNode(Node):
    """H1 robot dance controller — upper body only, legs fixed to ground."""

    JOINT_NAMES = [
        'left_hip_yaw', 'left_hip_pitch', 'left_hip_roll',
        'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
        'right_hip_yaw', 'right_hip_pitch', 'right_hip_roll',
        'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
        'torso',
        'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw', 'left_elbow',
        'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw', 'right_elbow',
    ]

    LOCKED_LEGS = {
        'left_hip_yaw': 0.0, 'right_hip_yaw': 0.0,
        'left_hip_pitch': 0.0, 'right_hip_pitch': 0.0,
        'left_hip_roll': 0.0, 'right_hip_roll': 0.0,
        'left_knee': 0.0, 'right_knee': 0.0,
        'left_ankle_pitch': 0.0, 'right_ankle_pitch': 0.0,
        'left_ankle_roll': 0.0, 'right_ankle_roll': 0.0,
    }

    MOVES = {
        'stand': {
            'torso': 0.0,
            'left_shoulder_pitch': 0.0, 'right_shoulder_pitch': 0.0,
            'left_shoulder_roll': 0.0, 'right_shoulder_roll': 0.0,
            'left_shoulder_yaw': 0.0, 'right_shoulder_yaw': 0.0,
            'left_elbow': 0.0, 'right_elbow': 0.0,
        },
        'wave_right': {
            'right_shoulder_pitch': -1.5, 'right_shoulder_roll': -1.2,
            'right_elbow': -1.5,
        },
        'wave_left': {
            'left_shoulder_pitch': -1.5, 'left_shoulder_roll': 1.2,
            'left_elbow': -1.5,
        },
        'wave_both': {
            'left_shoulder_pitch': -1.5, 'right_shoulder_pitch': -1.5,
            'left_shoulder_roll': 1.2, 'right_shoulder_roll': -1.2,
            'left_elbow': -1.5, 'right_elbow': -1.5,
        },
        'hands_up': {
            'left_shoulder_pitch': -2.5, 'right_shoulder_pitch': -2.5,
            'left_shoulder_roll': 0.3, 'right_shoulder_roll': -0.3,
            'left_elbow': -1.5, 'right_elbow': -1.5,
        },
        'hands_front': {
            'left_shoulder_pitch': -1.5, 'right_shoulder_pitch': -1.5,
            'left_elbow': -0.3, 'right_elbow': -0.3,
        },
        'twist_left': {
            'torso': 0.4,
            'left_shoulder_pitch': -0.8, 'right_shoulder_pitch': -1.2,
            'left_elbow': -0.5, 'right_elbow': -0.5,
        },
        'twist_right': {
            'torso': -0.4,
            'left_shoulder_pitch': -1.2, 'right_shoulder_pitch': -0.8,
            'left_elbow': -0.5, 'right_elbow': -0.5,
        },
        'dab_left': {
            'torso': 0.2,
            'left_shoulder_pitch': -1.0, 'left_shoulder_roll': 1.5,
            'left_elbow': -2.0,
            'right_shoulder_pitch': -1.5, 'right_shoulder_roll': -0.5,
            'right_elbow': -0.2,
        },
        'dab_right': {
            'torso': -0.2,
            'right_shoulder_pitch': -1.0, 'right_shoulder_roll': -1.5,
            'right_elbow': -2.0,
            'left_shoulder_pitch': -1.5, 'left_shoulder_roll': 0.5,
            'left_elbow': -0.2,
        },
        'clap_high': {
            'left_shoulder_pitch': -2.0, 'right_shoulder_pitch': -2.0,
            'left_shoulder_roll': 0.1, 'right_shoulder_roll': -0.1,
            'left_elbow': -0.5, 'right_elbow': -0.5,
        },
        'muscle_flex': {
            'left_shoulder_pitch': -1.2, 'right_shoulder_pitch': -1.2,
            'left_shoulder_roll': 1.5, 'right_shoulder_roll': -1.5,
            'left_elbow': -2.0, 'right_elbow': -2.0,
        },
    }

    # Moves safe for beat-synced dancing (no 'stand' — that's the reset)
    DANCE_MOVES = [
        'wave_right', 'wave_left', 'wave_both', 'hands_up', 'hands_front',
        'twist_left', 'twist_right', 'dab_left', 'dab_right',
        'clap_high', 'muscle_flex',
    ]

    def __init__(self):
        super().__init__('dance_gui')

        self.joint_index = {name: i for i, name in enumerate(self.JOINT_NAMES)}
        self.joint_pubs = {
            name: self.create_publisher(Float64, f'/robot/{name}_joint/cmd_pos', 10)
            for name in self.JOINT_NAMES
        }

        self.joint_positions = np.zeros(len(self.JOINT_NAMES))
        for joint, val in self.LOCKED_LEGS.items():
            self.joint_positions[self.joint_index[joint]] = val

        # Music state
        self.audio_path = None
        self.beat_times = []
        self.is_dancing = False

        self.create_timer(0.02, self._publish_joints)
        self.get_logger().info('Dance GUI node ready (upper body only).')

    def _publish_joints(self):
        for joint, val in self.LOCKED_LEGS.items():
            self.joint_positions[self.joint_index[joint]] = val
        for i, name in enumerate(self.JOINT_NAMES):
            msg = Float64()
            msg.data = float(self.joint_positions[i])
            self.joint_pubs[name].publish(msg)

    def perform_move(self, move_name, duration=2.0, reset=False):
        self.get_logger().info(f'Move: {move_name}')
        move = self.MOVES.get(move_name, {})

        start = self.joint_positions.copy()
        target = self.joint_positions.copy()

        upper_joints = [j for j in self.JOINT_NAMES if j not in self.LOCKED_LEGS]
        for j in upper_joints:
            target[self.joint_index[j]] = move.get(j, 0.0)

        steps = int(duration * 50)
        for step in range(steps):
            t = (step + 1) / steps
            alpha = 0.5 * (1 - math.cos(math.pi * t))
            self.joint_positions = start * (1 - alpha) + target * alpha
            time.sleep(0.02)

        self.joint_positions = target

        if reset:
            time.sleep(0.3)
            self.perform_move('stand', duration=1.0)

    def perform_routine(self):
        self.get_logger().info('Starting dance routine!')
        sequence = [
            ('hands_up',    1.2), ('stand', 0.6),
            ('twist_left',  0.8), ('twist_right', 0.8),
            ('twist_left',  0.8), ('twist_right', 0.8),
            ('stand',       0.5), ('wave_both', 1.0),
            ('stand',       0.5), ('dab_right', 0.8),
            ('stand',       0.4), ('dab_left', 0.8),
            ('stand',       0.4), ('muscle_flex', 1.0),
            ('stand',       0.5), ('clap_high', 0.6),
            ('hands_front', 0.6), ('clap_high', 0.6),
            ('hands_front', 0.6), ('stand', 0.5),
            ('wave_right',  0.8), ('wave_left', 0.8),
            ('hands_up',    1.0), ('stand', 1.0),
        ]
        for move, dur in sequence:
            self.perform_move(move, dur)

    # ── Music features ──────────────────────────────────────────────────

    def load_music(self):
        """Open a file dialog, load audio, and detect beats."""
        path = filedialog.askopenfilename(
            title='Select a music file',
            filetypes=[('Audio Files', '*.wav *.mp3 *.ogg *.flac')],
        )
        if not path:
            return
        self.audio_path = path
        self.get_logger().info(f'Loading: {path}')
        self.beat_times = detect_beats(path)
        self.get_logger().info(f'Detected {len(self.beat_times)} beats.')

    def dance_to_music(self):
        """Play loaded music and trigger moves on each beat."""
        if not self.audio_path:
            self.get_logger().warn('No music loaded!')
            return
        if not pygame:
            self.get_logger().error('pygame not installed.')
            return

        pygame.mixer.init()
        pygame.mixer.music.load(self.audio_path)
        pygame.mixer.music.play()
        self.is_dancing = True
        self.get_logger().info('Dancing to music!')

        start_time = time.time()
        beat_idx = 0
        last_move = 'stand'

        while (pygame.mixer.music.get_busy()
               and self.is_dancing
               and beat_idx < len(self.beat_times)):

            elapsed = time.time() - start_time

            if elapsed >= self.beat_times[beat_idx]:
                # Pick a move that's different from the last one
                move = random.choice(self.DANCE_MOVES)
                while move == last_move:
                    move = random.choice(self.DANCE_MOVES)
                last_move = move

                # Figure out how long until the next beat
                if beat_idx + 1 < len(self.beat_times):
                    dur = self.beat_times[beat_idx + 1] - self.beat_times[beat_idx]
                    dur = max(0.3, min(dur * 0.8, 1.5))
                else:
                    dur = 0.6

                # Every 4th beat, return to stand briefly
                if beat_idx % 4 == 3:
                    move = 'stand'
                    dur = max(0.3, dur)

                threading.Thread(
                    target=self.perform_move, args=(move, dur),
                    daemon=True
                ).start()

                beat_idx += 1

            time.sleep(0.01)

        # Return to stand when music ends
        self.is_dancing = False
        self.perform_move('stand', 1.0)
        self.get_logger().info('Music finished.')

    def stop_music(self):
        """Stop music playback and dancing."""
        self.is_dancing = False
        if pygame and pygame.mixer.get_init():
            pygame.mixer.music.stop()
        self.get_logger().info('Stopped.')


# ── GUI ─────────────────────────────────────────────────────────────────

def build_gui(node):
    root = tk.Tk()
    root.title('H1 Dance Controller')
    root.geometry('280x620')

    ttk.Label(root, text='H1 Dance Controller', font=('Arial', 12, 'bold')).pack(pady=8)

    # Individual move buttons
    buttons = [
        ('Stand',        'stand'),
        ('Wave Right',   'wave_right'),
        ('Wave Left',    'wave_left'),
        ('Wave Both',    'wave_both'),
        ('Hands Up',     'hands_up'),
        ('Hands Front',  'hands_front'),
        ('Twist Left',   'twist_left'),
        ('Twist Right',  'twist_right'),
        ('Dab Left',     'dab_left'),
        ('Dab Right',    'dab_right'),
        ('Muscle Flex',  'muscle_flex'),
    ]
    for label, move in buttons:
        ttk.Button(
            root, text=label,
            command=lambda m=move: threading.Thread(
                target=node.perform_move, args=(m, 1.5, True), daemon=True
            ).start(),
        ).pack(pady=1, fill='x', padx=20)

    ttk.Separator(root, orient='horizontal').pack(fill='x', pady=6)

    # Routine button
    ttk.Button(
        root, text='★ Dance Routine ★',
        command=lambda: threading.Thread(
            target=node.perform_routine, daemon=True
        ).start(),
    ).pack(pady=3, fill='x', padx=20)

    ttk.Separator(root, orient='horizontal').pack(fill='x', pady=6)

    # Music controls
    ttk.Label(root, text='♫ Music Mode', font=('Arial', 10, 'bold')).pack(pady=2)

    ttk.Button(
        root, text='Load Music',
        command=node.load_music,
    ).pack(pady=2, fill='x', padx=20)

    ttk.Button(
        root, text='▶ Dance to Music',
        command=lambda: threading.Thread(
            target=node.dance_to_music, daemon=True
        ).start(),
    ).pack(pady=2, fill='x', padx=20)

    ttk.Button(
        root, text='■ Stop',
        command=node.stop_music,
    ).pack(pady=2, fill='x', padx=20)

    root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = DanceGuiNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        build_gui(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
