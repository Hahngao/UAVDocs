
# =============================================================================
# 功能说明：
#   本脚本用于自动生成 Docsify 文档站点的侧边栏文件（_sidebar.md）。
#
# 主要功能：
#   1. 扫描 docs/ 目录下的所有 Markdown 文件及子文件夹；
#   2. 从每个 Markdown 文件的第一个标题（# 开头）中提取页面标题；
#   3. 读取 docs/README.md 中的链接顺序，按该顺序排列根目录下的 md 文件；
#   4. 遍历所有子文件夹，递归收集其中的 md 文件，按标题字母序排列；
#   5. 为每个子文件夹自动生成或覆盖 readme.md，内容为该文件夹的文件索引；
#   6. 将所有根文件与子文件夹的条目写入 docs/_sidebar.md，形成完整侧边栏结构。
#
# 适用场景：
#   - 使用 Docsify 搭建的静态文档站点
#   - 文档文件数量较多、需要自动维护侧边栏的项目
#
# 注意事项：
#   - 以下文件不会被纳入侧边栏：_sidebar.md、_navbar.md、.nojekyll、README.md、index.html
#   - 每次运行会覆盖已有的 _sidebar.md 和各子文件夹的 readme.md
#   - 脚本需与 docs/ 目录处于同一父目录下
# =============================================================================

import html
import os
import re

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.join(BASE_DIR, "docs")
README_PATH = os.path.join(ROOT_DIR, "README.md")
SIDEBAR_PATH = os.path.join(ROOT_DIR, "_sidebar.md")

IGNORE_FILES = ["_sidebar.md", "_navbar.md", ".nojekyll", "README.md", "index.html"]

HTML_TAG_RE = re.compile(r"<[^>]+>")
HEADER_RE = re.compile(r"^(#+)\s*(.*?)\s*$")

def normalize_title(title):
    title = html.unescape(title)
    title = HTML_TAG_RE.sub("", title)
    title = re.sub(r"\s+", " ", title)
    return title.strip()

def get_title(filepath):
    """Extract title from the first meaningful header in the file."""
    fallback_title = os.path.splitext(os.path.basename(filepath))[0]
    try:
        first_header_title = None
        leading_h1_titles = []

        with open(filepath, 'r', encoding='utf-8') as f:
            for line in f:
                stripped_line = line.strip()
                if not stripped_line:
                    continue

                header_match = HEADER_RE.match(stripped_line)
                if header_match:
                    header_level = len(header_match.group(1))
                    title = normalize_title(header_match.group(2))
                    if not title:
                        continue

                    if first_header_title is None:
                        first_header_title = title

                    if header_level == 1:
                        if first_header_title == title and not leading_h1_titles:
                            leading_h1_titles.append(title)
                            continue

                        if leading_h1_titles:
                            leading_h1_titles.append(title)
                            continue

                    if leading_h1_titles:
                        break
                    continue

                if leading_h1_titles:
                    break
    except Exception as e:
        print(f"Error reading {filepath}: {e}")

    if len(leading_h1_titles) == 1:
        return leading_h1_titles[0]

    if len(leading_h1_titles) > 1:
        return fallback_title

    if first_header_title:
        return first_header_title

    return fallback_title

def get_root_folders():
    try:
        folders = [
            name for name in os.listdir(ROOT_DIR)
            if os.path.isdir(os.path.join(ROOT_DIR, name)) and not name.startswith('.')
        ]
    except FileNotFoundError:
        print(f"Directory not found: {ROOT_DIR}")
        return []
    return sorted(folders)

def get_root_files_order():
    try:
        with open(README_PATH, 'r', encoding='utf-8') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"README not found: {README_PATH}")
        return []

    order = []
    for link in re.findall(r'\[[^\]]+\]\(([^)]+)\)', content):
        normalized_link = link.strip().replace('\\', '/')
        filename = os.path.basename(normalized_link)

        if '/' in normalized_link or not filename.lower().endswith('.md'):
            continue

        if filename in IGNORE_FILES or filename in order:
            continue

        order.append(filename)

    return order

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
    root_folders = get_root_folders()
    root_files_order = get_root_files_order()
    
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
    
    for name in root_files_order:
        # Case insensitive check
        found = next((f for f in remaining_files if f.lower() == name.lower()), None)
        if found:
            sorted_root_files.append(found)
            remaining_files.remove(found)
    
    sorted_root_files.extend(sorted(remaining_files))
    
    # Add root files to sidebar
    for filename in sorted_root_files:
        filepath = os.path.join(ROOT_DIR, filename)
        title = get_title(filepath)
        # Use bold for top level items as per existing style
        sidebar_lines.append(f"- [**{title}**]({filename})")

    # 2. Process Special Folders
    for folder in root_folders:
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
        # Sort files by title
        files.sort(key=lambda x: x['title'])
        
        folder_readme_content = [f"# {folder}\n"]
        
        for file_info in files:
            # Add to sidebar
            sidebar_lines.append(f"  - [{file_info['title']}]({file_info['path']})")
            # Add to folder readme
            folder_readme_content.append(f"- [{file_info['title']}]({file_info['path']})")
            
        # Update folder readme
        with open(readme_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(folder_readme_content))
            print(f"Updated {readme_path}")
            
    # Write Sidebar
    with open(SIDEBAR_PATH, 'w', encoding='utf-8') as f:
        f.write('\n'.join(sidebar_lines))

    print(f"Updated {SIDEBAR_PATH}")

if __name__ == "__main__":
    main()
