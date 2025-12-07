# 🚀 Quick Start Guide: Physical AI Textbook with Docusaurus

## Prerequisites
- Node.js 18+ installed
- Git installed
- GitHub account
- Basic terminal knowledge

## Step-by-Step Setup

### 1️⃣ Initialize Docusaurus Project

```bash
# Create project directory
mkdir physical-ai-textbook
cd physical-ai-textbook

# Initialize Docusaurus with TypeScript
npx create-docusaurus@latest my-book classic --typescript

# Navigate to book directory
cd my-book
```

### 2️⃣ Replace Configuration Files

Replace the generated files with our custom configs:

```bash
# Copy docusaurus.config.js (from artifact)
# Copy sidebars.js (from artifact)
```

**Important:** Update these values in `docusaurus.config.js`:
- `url`: Your GitHub Pages URL
- `baseUrl`: Your repo name
- `organizationName`: Your GitHub username
- `projectName`: Your repo name

### 3️⃣ Create Folder Structure

```bash
# Run the structure creation script
chmod +x create-book-structure.sh
./create-book-structure.sh
```

Or manually create folders:

```bash
cd docs

# Create all module folders
mkdir -p getting-started module-1 module-2 module-3 module-4 humanoid capstone appendix

# Create files (example for module-1)
touch module-1/introduction.md
touch module-1/ros2-architecture.md
# ... (repeat for all chapters)
```

### 4️⃣ Add Main Content Files

Copy these files into the docs folder:
- `intro.md` - Main landing page
- `module-1/introduction.md` - Module 1 intro

### 5️⃣ Test Locally

```bash
# Install dependencies
npm install

# Start development server
npm start
```

Your book should open at `http://localhost:3000`

### 6️⃣ Set Up GitHub Repository

```bash
# Initialize git
git init
git add .
git commit -m "Initial commit: Physical AI textbook structure"

# Create repo on GitHub (via web interface)
# Then push:
git remote add origin https://github.com/YOUR-USERNAME/physical-ai-textbook.git
git branch -M main
git push -u origin main
```

### 7️⃣ Deploy to GitHub Pages

```bash
# Build the site
npm run build

# Deploy
npm run deploy
```

Visit: `https://YOUR-USERNAME.github.io/physical-ai-textbook/`

## Using Spec-Kit Plus with Claude Code

### Setup Spec-Kit Plus

```bash
# Clone Spec-Kit Plus
cd ..  # Go back to parent directory
git clone https://github.com/panaversity/spec-kit-plus/
cd spec-kit-plus

# Install dependencies
npm install
```

### Create Spec Files

Create `specs/` folder in your project:

```bash
mkdir -p specs
```

Add spec files like `module-1-ros2.spec.md` (see artifact)

### Generate Content with Claude Code

```bash
# Configure Claude Code for Gemini
export GEMINI_API_KEY="your-api-key-here"

# Run Claude Code with spec
claude --model gemini-2.0-flash-exp \
       --spec specs/module-1-ros2.spec.md \
       --output docs/module-1/
```

## Workflow for Content Creation

### Option A: Manual with Claude Conversations
1. Copy chapter outline from sidebars.js
2. Ask Claude to write each chapter
3. Save responses as .md files in docs/
4. Test locally with `npm start`
5. Commit and deploy

### Option B: Automated with Spec-Kit Plus
1. Write detailed spec file
2. Run Claude Code with spec
3. Review generated content
4. Make adjustments if needed
5. Deploy

### Option C: Hybrid Approach (Recommended)
1. Use specs for structure and boilerplate
2. Use Claude conversations for specialized content
3. Edit and refine manually
4. Deploy incrementally

## File Naming Convention

Follow this pattern:
```
docs/
├── intro.md                          # Main intro
├── getting-started/
│   ├── overview.md                   # Use descriptive names
│   └── prerequisites.md
├── module-1/
│   ├── introduction.md               # Always start with intro
│   ├── ros2-architecture.md          # Kebab-case
│   └── project-ros2-package.md
```

## Content Structure Template

Every chapter should follow this structure:

```markdown
---
sidebar_position: 1
---

# Chapter Title

Brief introduction paragraph...

## 🎯 Learning Objectives
- Objective 1
- Objective 2

## Main Content Sections
### Subsection 1
Content...

### Subsection 2
Content...

## 💡 Key Takeaways
- Summary point 1
- Summary point 2

## 🏋️ Practice Exercise
Exercise description...

## 📚 Further Reading
- Resource 1
- Resource 2

---

**Next:** [Next Chapter](./next-chapter.md)
```

## Deployment Checklist

Before deploying:
- [ ] All internal links work
- [ ] Code examples are tested
- [ ] Images are in `/static/img/`
- [ ] No broken external links
- [ ] Sidebars navigation is correct
- [ ] Build succeeds: `npm run build`
- [ ] Test locally: `npm start`

## Common Issues & Solutions

### Issue: Deploy fails
```bash
# Solution: Check GitHub repo settings
# Ensure GitHub Pages source is set to gh-pages branch
```

### Issue: Sidebar doesn't show
```bash
# Solution: Check frontmatter in .md files
---
sidebar_position: 1
---
```

### Issue: Build errors
```bash
# Solution: Clear cache and rebuild
npm run clear
npm run build
```

## Next Steps

1. ✅ Complete basic structure
2. ✅ Write Module 1 content
3. ⏳ Add RAG chatbot (Phase 3)
4. ⏳ Implement Better-Auth (Bonus)
5. ⏳ Add personalization features (Bonus)

## Resources

- [Docusaurus Docs](https://docusaurus.io/)
- [Spec-Kit Plus](https://github.com/panaversity/spec-kit-plus/)
- [Claude Code](https://www.claude.com/product/claude-code)
- [Markdown Guide](https://www.markdownguide.org/)

## Support

If you get stuck:
1. Check Docusaurus documentation
2. Review spec file examples
3. Ask in hackathon Discord/WhatsApp
4. Use Claude for debugging

---

**Ready to create amazing content!** 🚀