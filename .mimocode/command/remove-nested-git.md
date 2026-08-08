---
description: "删除 src/ 子包中的嵌套 .git 目录，保留外层工作空间 git 不受影响。用法: remove-nested-git <子包路径>"
---

# 删除子包嵌套 .git

用户要删除 `$ARGUMENTS` 中的 `.git` 目录，同时保留外层工作空间的 git 不受影响。

## 步骤

1. **确认路径存在**：`ls -la $ARGUMENTS/.git` — 确认嵌套 .git 目录存在。
2. **检查是否为 submodule**：`cat <外层>/.gitmodules` 和 `git -C <外层> ls-files --error-unmatch <子包路径>` — 如果是 submodule，先 `git submodule deinit` 再删除。
3. **删除**：`rm -rf $ARGUMENTS/.git`
4. **验证**：`test -d $ARGUMENTS/.git && echo "还在" || echo "已删除"`
5. **确认外层 git 正常**：`git -C <外层工作空间根目录> status`

## 注意事项

- 如果子包中有 `.gitignore`，保留它（不影响外层）。
- 如果用户同时提供了多个子包路径，依次处理。
- 用户的典型表述："帮我删除 src/xxx 中的 git 然后外面的大 git 不要动"
