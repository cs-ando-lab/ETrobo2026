# 開発ガイドライン (CONTRIBUTING.md)

ここでは、リポジトリのクローンからmainブランチへのマージまでの一連の流れを説明します。
Windows版VScodeを使用した説明を行いますが、他のOSやエディタでも大体同じです。
開発環境の構築は既に済んでいるものとします。

## 1. 開発プロジェクトのクローン

### WSL上でETrobo環境を開く

環境構築をした際にダウンロードした`Start ETrobo SPIKE-RT.cmd`を実行する。

見つからない場合には、VScodeを起動し、左下の`リモートウィンドウを開く` -> `WSLへの接続` -> `フォルダを開く` をクリックし、etrobo環境を選択する。

### リポジトリをクローン

下記のコマンドを実行し、リポジトリをクローンしてください。

```
# workspaceフォルダに移動
cd workspace
# リポジトリをクローン
git clone https://github.com/botamochi-dev/ETrobo2026_code.git
# クローンしたリポジトリに移動
cd ETrobo2026_code
# クローンしたフォルダをVScodeで開く
code .
```

#### Gitのコミット情報を設定する

コミット履歴として記録される名前とメールを登録します。
既に設定している場合もありますが、この情報は全世界に公開されてしまうため、プライバシー保護の観点から大学のメールアドレスなどを設定している場合にはGitHubが提供するダミーのメーアドレスに変更しておきましょう。

自分の専用アドレスを確認したい場合は、以下の手順で確認できます。

1. GitHubの右上アイコンから Settings を開く。

2. 左メニューの Emails を選択。

3. Keep my email addresses private にチェックを入れると、そのすぐ下に {id}+{username}@users.noreply.github.com という形式のアドレスが表示されます。

```
git config user.name <GitHubのユーザー名>
git config user.email <GitHub提供の非公開メールアドレス>
```

## 2. コードの変更

### ブランチ作成

コードの変更を始める前には、**必ず**ブランチを作成します。

1.  VScodeのフッター左に表示されている赤枠の場所をクリック(特になにもしていなければ`main`と表示されているはず)
    ![](image/branch.png)

2.  `新しいブランチを作成`をクリックし、作成するブランチ名を入力する。
    ブランチの命名は[Git / GitHub 運用フロー](../Docs/GIT_WORKFLOW.md)を参考にしてください。

    ![](image/branch-select.png)
    
    ブランチが正常に作成されると左下の表記が作成したブランチ名になります。
    
    ![](image/branch-after.png)

### コードの変更

コードの変更やファイルの追加・削除を行います。

### コミット

1. VScodeの左側タブの赤枠をクリックすると、そのブランチで変更したファイル一覧が表示されます。
   ![](image/sourcetree.png)

2. コミットの前段階として、変更したファイルをステージングという状態にします。

   ![](image/stage.png)

3. 赤枠の部分にコミットメッセージを入力し、コミットボタンを押します。コミットルールは[Git / GitHub 運用フロー](../Docs/GIT_WORKFLOW.md)を参考にしてください。

   ![](image/commit-message.png)

### プッシュ

コミットを行うと下記のように`Branchの発行`と表示されるのでこれをクリックします。
これによりローカルブランチをリモートにも反映します。

![](image/push.png)

## 3. PRの作成・マージ

### PR作成

ブランチの発行後に https://github.com/botamochi-dev/ETrobo2026_code にアクセスすると下記のようにブランチがプッシュされた通知が来ます。この`Compare&pull request`をクリックします。
![](image/PR-notice.png)

すると、このようにプルリクエストの作成画面が表示されます。タイトルや説明などがある場合には追加し、`Create pull request`をクリックします。
![](image/PR-create.png)

### mainブランチへのマージ

`Merge pull request` を押します。
