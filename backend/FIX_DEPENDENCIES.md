# Fix Dependencies Issues - Step by Step

## Problem 1: Virtual Environment Not Properly Activated
## Problem 2: Missing tf-keras Package

## Solution:

### Step 1: Virtual Environment Properly Activate Karo

```bash
# Backend folder me jao
cd C:\Users\Hamna\Desktop\Humanoid_robotics\Physical-AI-Humanoid-Robotics-Book\backend

# Virtual environment deactivate karo (agar already active hai)
deactivate

# Phir activate karo
# Windows me:
myenv\Scripts\activate

# Ya phir full path:
C:\Users\Hamna\Desktop\Humanoid_robotics\Physical-AI-Humanoid-Robotics-Book\backend\myenv\Scripts\activate
```

### Step 2: Verify Virtual Environment Active Hai

Activate ke baad prompt me `(myenv)` dikhna chahiye:

```bash
(myenv) C:\Users\Hamna\Desktop\...\backend>
```

### Step 3: Python Path Check Karo

```bash
# Ye command run karo - virtual environment ka Python dikhna chahiye
where python
# Output me myenv\Scripts\python.exe dikhna chahiye
```

### Step 4: Missing Packages Install Karo

```bash
# Virtual environment me ho (myenv) dikhna chahiye
pip install tf-keras
pip install pyyaml
```

### Step 5: All Dependencies Reinstall Karo (Optional but Recommended)

```bash
# Virtual environment me ho
pip install --upgrade pip
pip install -r requirements.txt
pip install tf-keras
```

### Step 6: Verify Installation

```bash
# Check karo ki packages install hain
python -c "import yaml; print('yaml OK')"
python -c "import tf_keras; print('tf-keras OK')"
python -c "from sentence_transformers import SentenceTransformer; print('sentence-transformers OK')"
```

## Alternative: Fresh Virtual Environment

Agar phir bhi issues aaye, fresh virtual environment banao:

```bash
# Old environment delete karo (optional)
# cd backend
# rmdir /s myenv

# New virtual environment banao
cd backend
python -m venv myenv

# Activate karo
myenv\Scripts\activate

# Dependencies install karo
pip install --upgrade pip
pip install -r requirements.txt
pip install tf-keras
```

## Quick Fix Commands (Copy Paste)

```bash
# 1. Backend folder me jao
cd C:\Users\Hamna\Desktop\Humanoid_robotics\Physical-AI-Humanoid-Robotics-Book\backend

# 2. Virtual environment activate karo
myenv\Scripts\activate

# 3. Missing packages install karo
pip install tf-keras pyyaml

# 4. Verify
python -c "import yaml; import tf_keras; print('All OK!')"
```

## After Fix - Run Content Ingestion

```bash
# Virtual environment me ho (myenv dikhna chahiye)
python scripts/ingest_content.py --source ../book_frontend/docs --create-collection --batch-size 50
```

