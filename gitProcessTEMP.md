# 一、Git流程备份（开发中...）
## 0、管理者
- `git branch develop`
- `git push -u origin develop`
## 1、开始
- 创建一个workspace文件夹，点击进入
- 打开git，执行`git clone https://gitee.com/causehhc/employee-home.git`
- `cd e*`
## 2、同步并新建xxx功能分支
- 同步远程develop仓库`git pull origin develop`
- 新建分支`git checkout -b feature/xxx origin/develop`
## 2-1、已有xxx分支，对该分支进行更新
- `git pull origin develop`
- `git checkout feature/xxx`
- `git merge develop`
## 2-3、强制pull develop（危险操作，必须手动备份自己做过的更改）
- 强制重置本地develop`git reset --hard develop`***危险！！！***
- 然后把远程develop pull下来`git pull origin develop`
## 3、开发
- TODO
## 4、add&commit
- `git add .`
- `git commit -m "updated xxx"`
## 5、合并到develop
- 先拉取develop中的代码`git pull origin develop`
- 切到develop分支`git checkout develop`
- 合并<feature/xxx>中的代码到develop中`git merge feature/xxx`
- 提交到develop远程分支上`git push`
- 删除本地的分支`git branch -d feature/xxx`
## 6、发布到master
- 建立发布准备分支`git checkout -b release/tag0.1 origin/develop`
- 先拉取master中的代码`git pull origin master`
- 切到master分支`git checkout master`
- 将release分支合到master上`git merge release/tag0.1`
- 将合完的代码提交到远程master`git push`
- 切到develop分支`git checkout develop`
- 将release分支上的代码合到develop分支上`git merge release/tag0.1`
- 合完的代码推送到远程的develop分支`git push`
- 删除本地release分支`git branch -d release/tag0.1`
