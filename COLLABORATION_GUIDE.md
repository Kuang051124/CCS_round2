# 团队协作开发指南

## 项目信息

| 项目 | 详情 |
|------|------|
| 远程仓库 | https://github.com/Kuang051124/CCS_round2 |
| 开发环境 | TI CCS / Theia IDE |
| 芯片平台 | MSPM0G3507 |
| 分支策略 | main 主分支 |

---

## 一、首次加入（队友必读）

### 1.1 安装 Git

去 [git-scm.com](https://git-scm.com) 下载安装。

安装后打开终端（Git Bash 或 CMD），配置自己的身份：

```bash
git config --global user.name "你的姓名"
git config --global user.email "你的邮箱"
```

### 1.2 配置代理（国内必备）

由于 GitHub 在国内被墙，需要配置梯子代理。找到你的梯子端口号（Clash 常见 7890/7897，v2rayN 常见 10809）：

```bash
git config --global http.proxy http://127.0.0.1:你的端口
git config --global https.proxy http://127.0.0.1:你的端口
```

> 梯子没开时 push/pull 会超时，取消代理：`git config --global --unset http.proxy`

### 1.3 克隆项目

```bash
cd 你的工作目录
git clone https://github.com/Kuang051124/CCS_round2.git
cd CCS_round2
```

### 1.4 用 CCS 打开

1. 打开 TI CCS / Theia IDE
2. File → Import → CCS Projects → 选择刚 clone 的文件夹
3. 项目即可编译运行

---

## 二、日常开发流程

每天按这个步骤来，永远不出错：

```
开 工               下 班
 ↓                  ↑
git pull    →    写代码    →    git add .  →  git commit  →  git push
```

### 开工第一步：拉取最新代码

```bash
git pull
```

> ⚠️ **每天开工必须先做这一步！** 否则可能会和队友的代码冲突。

### 写代码

正常在 CCS 里编辑代码，调试，烧录测试。

### 下班前：提交并推送

```bash
# 1. 查看自己改了什么
git status

# 2. 暂存所有修改
git add .

# 3. 提交到本地（写清楚做了什么）
git commit -m "修改：优化PID参数，修复电机抖动"

# 4. 推送到远程仓库（队友就能看到了）
git push
```

---

## 三、分支协作（推荐）

多人同时开发不同功能时，用分支互不干扰：

```bash
# 创建并切换到自己的分支
git checkout -b feature/你的功能名

# 在分支上正常开发...
git add .
git commit -m "在蓝牙模块添加新指令"
git push -u origin feature/你的功能名

# 功能完成后，切回 main 合并
git checkout main
git pull
git merge feature/你的功能名
git push
```

### 分支命名规范

| 类型 | 示例 |
|------|------|
| 新功能 | `feature/bluetooth-at` |
| 修Bug | `fix/motor-jitter` |
| 测试 | `test/pid-tuning` |

---

## 四、冲突处理

当两个人同时改了同一个文件，`git pull` 时会报 **CONFLICT**：

```bash
git pull
# Auto-merging main.c
# CONFLICT (content): Merge conflict in main.c
```

### 解决方法

打开冲突文件，搜索 `<<<<<<<`，你会看到：

```c
<<<<<<< HEAD
// 你写的代码
void motor_start(void) {
    TB6612_Init(100);
}
=======
// 队友写的代码
void motor_start(void) {
    TB6612_Init(80);
}
>>>>>>> origin/main
```

**手动决定保留哪个**（或合并两者），删掉标记符号：

```c
// 合并后的代码
void motor_start(void) {
    TB6612_Init(90);   // 取中间值，双方确认
}
```

然后：

```bash
git add .
git commit -m "解决 main.c 冲突"
git push
```

> 💡 **预防冲突**：开工前先 `git pull`，同一个文件尽量分工明确，改前跟队友说一声。

---

## 五、注意事项

### 哪些文件要提交？
- ✅ `.c` / `.h` 源文件
- ✅ `.syscfg` 配置文件
- ✅ `.ccsproject` / `.cproject` 工程文件
- ✅ `.settings/` 工程设置
- ✅ `targetConfigs/` 调试配置

### 哪些文件不要提交？（.gitignore 已自动排除）
- ❌ `Debug/` 编译输出目录
- ❌ `*.o` `*.out` `*.hex` 编译产物
- ❌ `.rar` `.zip` 压缩包

### 通用规则
- 每次 commit 写清楚改了什么，别写 "update" 或 "123"
- 一个功能一次 commit，不要攒一大堆一起提交
- 遇到问题先 `git status` 看看当前状态

---

## 六、常用命令速查

| 命令 | 作用 |
|------|------|
| `git pull` | 拉取最新代码 |
| `git status` | 查看修改了哪些文件 |
| `git diff 文件名` | 查看文件具体改了什么 |
| `git add .` | 暂存所有修改 |
| `git commit -m "说明"` | 提交到本地仓库 |
| `git push` | 推送到 GitHub |
| `git log --oneline` | 查看提交历史 |
| `git branch` | 查看所有分支 |
| `git checkout -b 分支名` | 创建并切换分支 |
| `git checkout main` | 切回主分支 |
| `git stash` | 暂存当前修改（临时切任务用） |
| `git stash pop` | 恢复暂存的修改 |
| `git restore 文件名` | 撤销文件的修改（恢复到最后一次 commit） |
| `git reset --soft HEAD~1` | 撤销最后一次 commit（保留修改） |

---

## 七、出了问题怎么办？

| 情况 | 解决方法 |
|------|----------|
| push 被拒绝（队友刚推了） | `git pull` → 合并 → `git push` |
| 改乱了想重来 | `git checkout -- 文件名` 恢复单个文件 |
| commit 信息写错了 | `git commit --amend -m "新信息"` |
| 忘记 pull 就改了代码 | `git stash` → `git pull` → `git stash pop` |
| 想回到某个历史版本 | `git log` 找到版本号 → `git checkout 版本号` |
| 梯子没开 push 失败 | 开梯子，或者 `git config --global --unset http.proxy` 走 Gitee |
