# 关于远程存储库from github 官方

## 在这篇文章中

* [关于远程存储库](https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#about-remote-repositories)
* [创建远程存储库](https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#creating-remote-repositories)
* [选择远程存储库的 URL](https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#choosing-a-url-for-your-remote-repository)
* [使用 HTTPS URL 克隆](https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#cloning-with-https-urls)
* [使用 SSH URL 克隆](https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#cloning-with-ssh-urls)
* [使用 GitHub CLI 克隆](https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#cloning-with-github-cli)

GitHub 的协作开发方法依赖于将提交从本地存储库发布到 GitHub，供其他人查看、获取和更新。

## [关于远程存储库](https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#about-remote-repositories)

远程 URL 是 Git 对“存储代码的地方”的奇特表达方式。该 URL 可能是您在 GitHub 上的存储库，也可能是其他用户的分支，甚至是完全不同的服务器上的存储库。

您只能推送到两种类型的 URL 地址：

* 一个 HTTPS URL，例如 `https://github.com/user/repo.git`
* SSH URL，例如 `git@github.com:user/repo.git`

Git 将远程 URL 与名称相关联，默认远程通常称为 `origin`

## [创建远程存储库](https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#creating-remote-repositories)

您可以使用该命令将远程 URL 与名称进行匹配。 例如，您可以在命令行中键入以下内容：`git remote add`

```shell
git remote add origin <REMOTE_URL>
```

这会将名称与 .`origin``REMOTE_URL`建立联系

您可以使用该命令更改远程的 URL。`git remote set-url`

## [选择远程存储库的 URL](https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#choosing-a-url-for-your-remote-repository)

有几种方法可以克隆 GitHub.com 上的存储库。

当您在登录帐户时查看存储库时，可用于将项目克隆到计算机上的 URL 位于存储库详细信息下方。

有关设置或更改远程 URL 的信息，请参阅“[管理远程存储库](https://docs.github.com/en/get-started/getting-started-with-git/managing-remote-repositories)”。

## [使用 HTTPS URL 克隆](https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#cloning-with-https-urls)

克隆 URL 在所有存储库上都可用，无论可见性如何。 即使您位于防火墙或代理后面，克隆 URL 也可以工作。`https://``https://`

当您在命令行上使用 HTTPS URL 访问远程仓库时，Git 将要求您提供 GitHub 用户名和密码。当 Git 提示您输入密码时，请输入您的个人访问令牌。或者，可以使用凭据帮助程序，例如 Git 凭据管理器。删除了 Git 的基于密码的身份验证，转而采用更安全的身份验证方法。有关详细信息，请参阅“[管理个人访问令牌](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token)”。`git clone``git fetch``git pull``git push`

如果要访问使用 SAML SSO 的组织，并且使用个人访问令牌（经典），则还必须在进行身份验证之前授权个人访问令牌访问组织。有关详细信息，请参阅"[About authentication with SAML single sign-on](https://docs.github.com/en/authentication/authenticating-with-saml-single-sign-on/about-authentication-with-saml-single-sign-on)"和“[授权个人访问令牌以用于 SAML 单点登录](https://docs.github.com/en/authentication/authenticating-with-saml-single-sign-on/authorizing-a-personal-access-token-for-use-with-saml-single-sign-on)”。[](https://docs.github.com/en/authentication/authenticating-with-saml-single-sign-on/about-authentication-with-saml-single-sign-on)

 **提示** ：

* 您可以使用凭据帮助程序，以便 Git 每次与 GitHub 通信时都会记住您的 GitHub 凭据。有关详细信息，请参阅“[在 Git 中缓存 GitHub 凭据](https://docs.github.com/en/get-started/getting-started-with-git/caching-your-github-credentials-in-git)”。
* 要克隆仓库而不在命令行上向 GitHub 进行身份验证，您可以改用 GitHub Desktop 进行克隆。有关详细信息，请参阅“[将仓库从 GitHub 克隆到 GitHub Desktop](https://docs.github.com/en/desktop/adding-and-cloning-repositories/cloning-a-repository-from-github-to-github-desktop)”。

如果您希望使用 SSH，但无法通过端口 22 进行连接，则可以通过 HTTPS 端口使用 SSH。有关详细信息，请参阅“[通过 HTTPS 端口使用 SSH](https://docs.github.com/en/authentication/troubleshooting-ssh/using-ssh-over-the-https-port)”。

## [使用 SSH URL 克隆](https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#cloning-with-ssh-urls)

SSH URL 通过 SSH（一种安全协议）提供对 Git 存储库的访问。要使用这些 URL，您必须在计算机上生成 SSH 密钥对，并在 GitHub.com 上将**公**钥添加到您的帐户。有关详细信息，请参阅“[使用 SSH 连接到 GitHub](https://docs.github.com/en/authentication/connecting-to-github-with-ssh)”。

当您使用 SSH URL 、 、 或远程存储库时，系统将提示您输入密码，并且必须提供 SSH 密钥密码。有关详细信息，请参阅“使用 SSH 密钥密码”。`git clone``git fetch``git pull``git push`

如果您要访问使用 SAML 单点登录 （SSO） 的组织，则必须先授权 SSH 密钥访问该组织，然后才能进行身份验证。有关详细信息，请参阅 GitHub Enterprise Cloud 文档中的“关于使用 SAML 单点[登录进行身份验证](https://docs.github.com/en/enterprise-cloud@latest/authentication/authenticating-with-saml-single-sign-on/about-authentication-with-saml-single-sign-on)”和“[授权用于 SAML 单点登录的 SSH 密钥](https://docs.github.com/en/enterprise-cloud@latest/authentication/authenticating-with-saml-single-sign-on/authorizing-an-ssh-key-for-use-with-saml-single-sign-on)”。

 **提示** ： 您可以使用 SSH URL 将存储库克隆到计算机，也可以将其作为将代码部署到生产服务器的安全方法。您还可以将 SSH 代理转发与部署脚本一起使用，以避免在服务器上管理密钥。有关详细信息，请参阅“[使用 SSH 代理转发](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/using-ssh-agent-forwarding)”。

## [使用 GitHub CLI 克隆](https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#cloning-with-github-cli)

您还可以安装 GitHub CLI 以在终端中使用 GitHub 工作流。有关详细信息，请参阅“[关于 GitHub CLI](https://docs.github.com/en/github-cli/github-cli/about-github-cli)”
