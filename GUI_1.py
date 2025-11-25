import tkinter as tk
from tkinter import ttk, messagebox, filedialog


def on_submit():
    choice = selected_option.get()
    if choice == "Specify Targets":
        targets = get_targets()
        message = f"You selected: {choice}\nTargets: {targets}"
    elif choice == "Manual Mode":
        # include slider value in manual mode
        val = manual_var.get()
        message = f"You selected: {choice}\nManual value: {val:.3f}"
    else:
        message = f"You selected: {choice}"
    messagebox.showinfo("Selection", message)
    print(message)



def on_option_change(*_):
    # Show or hide the targets frame depending on selection
    sel = selected_option.get()
    if sel == "Specify Targets":
        targets_frame.pack(padx=12, pady=(6, 12), fill="x")
    else:
        targets_frame.pack_forget()
    # show manual slider only in Manual Mode
    if sel == "Manual Mode":
        manual_frame.pack(padx=12, pady=(6, 12), fill="x")
    else:
        manual_frame.pack_forget()


def get_targets():
    # Return list of selected targets (one per combobox row)
    return [var.get().strip() for (_row, var, combo) in target_entries if var.get().strip()]


def update_combobox_values(changed_var=None):
    """Update each combobox's available `values` list so that already-selected
    targets are removed from other comboboxes (prevents duplicates).
    If a combobox's current selection becomes unavailable, clear it.
    """
    # set of currently selected non-empty targets
    selected = {var.get().strip() for (_r, var, _c) in target_entries if var.get().strip()}
    for (_row, var, combo) in target_entries:
        current = var.get().strip()
        # allowed options: all targets except those selected by others
        others_selected = set(selected)
        if current:
            others_selected.discard(current)
        allowed = [v for v in target_values if v not in others_selected]
        # update combobox values
        combo['values'] = allowed
        # if current no longer valid, clear it
        if current and current not in allowed:
            var.set("")


root = tk.Tk()
root.title("Gantry Control")
# Main layout: left controls, right image panel
main_frame = ttk.Frame(root)
main_frame.pack(fill="both", expand=True, padx=6, pady=6)

# Left: controls frame
controls_frame = ttk.Frame(main_frame)
controls_frame.pack(side="left", fill="both", expand=True)

# Right: image frame (square display)
image_frame = ttk.Frame(main_frame, width=300)
image_frame.pack(side="right", fill="y", padx=(6, 0))

# Instruction label
label = ttk.Label(controls_frame, text="Choose one of the five options:")
label.pack(padx=12, pady=(12, 6))

# StringVar to hold the selected option (default to Manual Mode)
selected_option = tk.StringVar(value="Manual Mode")
selected_option.trace_add("write", on_option_change)

# Create 4 radiobuttons
options = ["Manual Mode", "All Targets", "Specify Targets", "All Even ID Targets", "Other"]
for opt in options:
    rb = ttk.Radiobutton(controls_frame, text=opt, value=opt, variable=selected_option)
    rb.pack(anchor="w", padx=20, pady=2)

# Frame that will contain manual mode controls (hidden unless 'Manual Mode' chosen)
manual_frame = ttk.Frame(controls_frame, relief="groove")

manual_label = ttk.Label(manual_frame, text="Manual control (0.0 - 1.0):")
manual_label.pack(anchor="w", padx=6, pady=(6, 4))

# DoubleVar for slider value
manual_var = tk.DoubleVar(value=0.0)

# horizontal slider between 0 and 1
manual_scale = ttk.Scale(manual_frame, from_=0.0, to=1.0, orient="horizontal", variable=manual_var,
                         command=lambda v: manual_value_label.config(text=f"{float(v):.3f}"))
manual_scale.pack(fill="x", padx=6)

manual_value_label = ttk.Label(manual_frame, text=f"{manual_var.get():.3f}")
manual_value_label.pack(anchor="e", padx=6, pady=(2, 6))

# Frame that will contain target-specific controls (hidden unless 'Specify Targets' chosen)
targets_frame = ttk.Frame(controls_frame, relief="groove")

targets_label = ttk.Label(targets_frame, text="Select one or more targets:")
targets_label.pack(anchor="w", padx=6, pady=(6, 4))

# Combobox values
target_values = [f"Target_{i}" for i in range(1, 9)]

# Frame holding the list of combobox rows
targets_list_frame = ttk.Frame(targets_frame)
targets_list_frame.pack(fill="x", padx=6)

target_entries = []  # list of tuples (row_frame, tk.StringVar, combobox_widget)


def add_target(selected=None):
    """Add a combobox row. If `selected` provided, set as initial selection."""
    idx = len(target_entries) + 1
    row = ttk.Frame(targets_list_frame)
    lbl = ttk.Label(row, text=f"Target {idx}:")
    var = tk.StringVar(value=selected if selected is not None else "")
    combo = ttk.Combobox(row, values=target_values, textvariable=var, state="readonly")
    # attach trace to refresh all combobox value lists when this variable changes
    var.trace_add("write", lambda *args, v=var: update_combobox_values(v))
    remove_btn = ttk.Button(row, text="Remove", width=8, command=lambda r=row: remove_target(r))
    lbl.pack(side="left")
    combo.pack(side="left", fill="x", expand=True, padx=(6, 6))
    remove_btn.pack(side="right")
    row.pack(fill="x", pady=4)
    target_entries.append((row, var, combo))


def remove_target(row_frame):
    # Remove a target row and relabel remaining rows
    for i, (row, var, combo) in enumerate(target_entries):
        if row is row_frame:
            row.destroy()
            target_entries.pop(i)
            break
    for i, (row, var, combo) in enumerate(target_entries, start=1):
        lbl = row.winfo_children()[0]
        lbl.config(text=f"Target {i}:")
    # refresh allowed values after removal
    update_combobox_values()


# Prepopulate with two combobox rows (like before)
add_target()
add_target()

# initialize combobox value lists to remove duplicates
update_combobox_values()

controls_row = ttk.Frame(targets_frame)
controls_row.pack(fill="x", padx=6, pady=6)
add_btn = ttk.Button(controls_row, text="Add target", command=lambda: add_target())
add_btn.pack(side="left")

# Buttons frame
btn_frame = ttk.Frame(controls_frame)
btn_frame.pack(padx=12, pady=12, fill="x")

submit_btn = ttk.Button(btn_frame, text="Submit", command=on_submit)
submit_btn.pack(side="left", expand=True)

quit_btn = ttk.Button(btn_frame, text="Quit", command=root.destroy)
quit_btn.pack(side="right")

# Ensure initial visibility is correct
on_option_change()

# --- Image panel contents ---
# Square canvas to display the targets image (starts with placeholder)
IMG_SIZE = 240
image_canvas = tk.Canvas(image_frame, width=IMG_SIZE, height=IMG_SIZE, bg="#f0f0f0", highlightthickness=1, highlightbackground="#999")
image_canvas.pack(padx=6, pady=6)
image_canvas.create_rectangle(4, 4, IMG_SIZE-4, IMG_SIZE-4, outline="#666")
image_canvas.create_text(IMG_SIZE//2, IMG_SIZE//2, text="Targets", fill="#666")

# Keep a reference to the currently displayed PhotoImage (if any)
target_image_ref = None


def set_target_image(path):
    """Load an image from `path` and display it in the right panel. Uses PIL if available."""
    global target_image_ref
    try:
        from PIL import Image, ImageTk
        img = Image.open(path)
        img = img.resize((IMG_SIZE, IMG_SIZE), Image.LANCZOS)
        target_image_ref = ImageTk.PhotoImage(img)
    except Exception:
        # fallback: try Tk PhotoImage (supports GIF/PNG on many builds)
        try:
            target_image_ref = tk.PhotoImage(file=path)
        except Exception as e:
            messagebox.showerror("Image load error", f"Could not load image:\n{e}")
            return
    image_canvas.delete("all")
    image_canvas.create_image(0, 0, anchor="nw", image=target_image_ref)


def on_load_image():
    path = filedialog.askopenfilename(title="Select target image",
                                      filetypes=[("Image files", "*.png;*.jpg;*.jpeg;*.gif;*.bmp"), ("All files", "*")])
    if path:
        set_target_image(path)


load_btn = ttk.Button(image_frame, text="Load image...", command=on_load_image)
load_btn.pack(padx=6, pady=(0, 10))

root.mainloop()