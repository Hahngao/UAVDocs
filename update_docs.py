import os

ROOT_DIR = r"d:\CodeProject\UAVDocs\docs"
SIDEBAR_PATH = os.path.join(ROOT_DIR, "_sidebar.md")

# Level 1 folders
FOLDERS = [
    "飞控开发教程",
    "地面站开发教程",
    "硬件开发教程",
    "应用开发教程"
]

# Preferred order for root files
ROOT_FILES_ORDER = [
    "intro.md",
    "systemoverview.md",
    "hardware_options.md",
    "quickstart.md",
    "api.md"
]

# Files to ignore in sidebar scanning
IGNORE_FILES = ["_sidebar.md", "_navbar.md", ".nojekyll", "README.md", "index.html"]

def get_title(filepath):
    """Extract title from the first header in the file."""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line.startswith('#'):
                    # Remove # and whitespace
                    return line.lstrip('#').strip()
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
    
    # Fallback to filename without extension
    return os.path.splitext(os.path.basename(filepath))[0]

def scan_folder(folder_path):
    """Recursively scan for .md files, excluding readme.md."""
    md_files = []
    for root, dirs, files in os.walk(folder_path):
        for file in files:
            if file.lower().endswith('.md') and file.lower() != 'readme.md':
                full_path = os.path.join(root, file)
                # Rel path from docs root for sidebar
                rel_path_from_docs = os.path.relpath(full_path, ROOT_DIR).replace('\\', '/')
                # Rel path from folder for readme
                rel_path_from_folder = os.path.relpath(full_path, folder_path).replace('\\', '/')
                
                title = get_title(full_path)
                md_files.append({
                    'title': title,
                    'path': rel_path_from_docs,
                    'local_path': rel_path_from_folder,
                    'filename': file
                })
    return md_files

def main():
    sidebar_lines = []
    
    # 1. Process Root Files
    # Get all md files in root
    try:
        root_files_all = [f for f in os.listdir(ROOT_DIR) 
                          if f.lower().endswith('.md') 
                          and os.path.isfile(os.path.join(ROOT_DIR, f))
                          and f not in IGNORE_FILES]
    except FileNotFoundError:
        print(f"Directory not found: {ROOT_DIR}")
        return

    # Sort root files: preferred ones first, then others alphabetically
    sorted_root_files = []
    remaining_files = root_files_all.copy()
    
    for name in ROOT_FILES_ORDER:
        # Case insensitive check
        found = next((f for f in remaining_files if f.lower() == name.lower()), None)
        if found:
            sorted_root_files.append(found)
            remaining_files.remove(found)
    
    # Append any other root files (excluding progress.md which we handle last)
    others = []
    progress_file = None
    for f in remaining_files:
        if f.lower() == "progress.md":
            progress_file = f
        else:
            others.append(f)
            
    sorted_root_files.extend(sorted(others))
    
    # Add root files to sidebar
    for filename in sorted_root_files:
        filepath = os.path.join(ROOT_DIR, filename)
        title = get_title(filepath)
        # Use bold for top level items as per existing style
        sidebar_lines.append(f"- [**{title}**]({filename})")

    # 2. Process Special Folders
    for folder in FOLDERS:
        folder_path = os.path.join(ROOT_DIR, folder)
        if not os.path.exists(folder_path):
            print(f"Folder not found, skipping: {folder}")
            continue
            
        # Ensure readme exists
        readme_path = os.path.join(folder_path, "readme.md")
            
        # Add folder to sidebar
        # Using folder name as title for the section header
        sidebar_lines.append(f"- [**{folder}**]({folder}/readme.md)")
        
        # Scan children
        files = scan_folder(folder_path)
        # Sort files by filename
        files.sort(key=lambda x: x['filename'])
        
        folder_readme_content = [f"# {folder}\n"]
        
        for file_info in files:
            # Add to sidebar
            sidebar_lines.append(f"  - [{file_info['title']}]({file_info['path']})")
            # Add to folder readme
            folder_readme_content.append(f"- [{file_info['title']}]({file_info['local_path']})")
            
        # Update folder readme
        with open(readme_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(folder_readme_content))
            print(f"Updated {readme_path}")
            
    # 3. Handle progress.md
    if progress_file:
        filepath = os.path.join(ROOT_DIR, progress_file)
        title = get_title(filepath)
        sidebar_lines.append(f"- [**{title}**]({progress_file})")

    # Write Sidebar
    with open(SIDEBAR_PATH, 'w', encoding='utf-8') as f:
        f.write('\n'.join(sidebar_lines))

    print(f"Updated {SIDEBAR_PATH}")

if __name__ == "__main__":
    main()
