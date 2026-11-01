"""Application launcher. Run with: py TrikiMotionController.py"""
from triki_controller.app import TrikiMotionApp
import tkinter as tk

def main():
    root = tk.Tk()
    TrikiMotionApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
