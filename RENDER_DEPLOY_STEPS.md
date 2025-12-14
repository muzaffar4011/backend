# Render Backend Deployment - Step by Step Guide

## Prerequisites Checklist

Before deploying, ensure you have:

- [ ] **Render Account**: Sign up at https://render.com (Free tier available)
- [ ] **GitHub Repository**: Code pushed to GitHub (hamnakh/Physical-AI-Humanoid-Robotics-Book)
- [ ] **OpenAI API Key**: Get from https://platform.openai.com/api-keys
- [ ] **Qdrant Cloud Account**: Sign up at https://cloud.qdrant.io (Free tier available)
- [ ] **PostgreSQL Database**: Use Render PostgreSQL (free) or Neon (free tier)

---

## Step 1: Set Up Qdrant Cloud (Vector Database)

### 1.1 Create Qdrant Account
1. Go to https://cloud.qdrant.io/
2. Sign up for free account
3. Verify email

### 1.2 Create Cluster
1. Dashboard → **Create Cluster**
2. Choose **Free Tier** (1 cluster free)
3. Select region (closest to you)
4. Create cluster

### 1.3 Get Credentials
1. Open your cluster
2. Copy **Cluster URL** (e.g., `https://xyz-1234.us-east4-0.gcp.cloud.qdrant.io`)
3. Go to **API Keys** → Create new API key → Copy it

### 1.4 Create Collection (Optional - Script will create it)
You can create manually or let the ingestion script create it:
```bash
curl -X PUT 'YOUR-CLUSTER-URL/collections/book_content' \
  -H 'api-key: YOUR-API-KEY' \
  -H 'Content-Type: application/json' \
  -d '{
    "vectors": {
      "size": 768,
      "distance": "Cosine"
    }
  }'
```

**Save these for Step 3:**
- ✅ QDRANT_URL
- ✅ QDRANT_API_KEY

---

## Step 2: Set Up Database (Choose One)

### Option A: Render PostgreSQL (Recommended - Easy)

1. **Render Dashboard** → **New +** → **PostgreSQL**
2. Configure:
   - **Name**: `rag-chatbot-db`
   - **Region**: Oregon (or nearest)
   - **Plan**: Free
3. Click **Create Database**
4. Wait for database to be ready (~2 min)
5. Copy **Internal Database URL** (looks like: `postgresql://user:pass@host:5432/dbname`)

### Option B: Neon PostgreSQL (Alternative)

1. Go to https://neon.tech/
2. Sign up → Create project
3. Copy **Connection String** (pooled recommended: contains `-pooler`)

**Save for Step 3:**
- ✅ DATABASE_URL

---

## Step 3: Deploy to Render

### 3.1 Connect GitHub Repository

1. Go to https://dashboard.render.com/
2. Click **New +** → **Blueprint**
3. Connect GitHub account (if not connected)
4. Select repository: **hamnakh/Physical-AI-Humanoid-Robotics-Book**
5. Render will detect `render.yaml` automatically
6. Click **Apply**

### 3.2 Configure Environment Variables

Render Dashboard me aapko service dikh raha hoga. Click karke **Environment** tab me ye variables add karo:

#### Required Secrets (Click "Add Secret" for each):

```bash
# OpenAI (Required)
OPENAI_API_KEY = sk-your-openai-api-key-here

# Qdrant (Required)
QDRANT_URL = https://your-cluster.qdrant.io
QDRANT_API_KEY = your-qdrant-api-key-here

# Database (Required)
DATABASE_URL = postgresql://user:password@host:5432/dbname
```

#### Already Set in render.yaml (Verify these are correct):

- ✅ `OPENAI_CHAT_MODEL` = gpt-4o-mini
- ✅ `OPENAI_TEMPERATURE` = 0.3
- ✅ `OPENAI_MAX_TOKENS` = 1000
- ✅ `EMBEDDING_MODEL_NAME` = BAAI/bge-base-en-v1.5
- ✅ `EMBEDDING_DEVICE` = cpu
- ✅ `EMBEDDING_BATCH_SIZE` = 16
- ✅ `QDRANT_COLLECTION_NAME` = book_content
- ✅ `QDRANT_VECTOR_SIZE` = 768
- ✅ `ENVIRONMENT` = production
- ✅ `CORS_ORIGINS` = http://localhost:3000,https://hamnakh.github.io
- ✅ `RATE_LIMIT_PER_MINUTE` = 10
- ✅ `LOG_LEVEL` = INFO

### 3.3 Start Deployment

1. **Environment variables** save karne ke baad
2. Click **Manual Deploy** → **Deploy latest commit**
3. Ya phir wait karo (Render auto-deploy karega on push)

### 3.4 Monitor Deployment

- **Events** tab me progress dekh sakte ho
- **Logs** tab me real-time logs
- Build time: ~5-10 minutes (first time)
- Model download: First request pe ~2-5 min extra

---

## Step 4: Run Database Migrations

### Option A: Using Render Shell (Recommended)

1. Render Dashboard → Your Service → **Shell** tab
2. Run:
```bash
cd backend
alembic upgrade head
```

### Option B: Using Local Terminal (If Render Shell not available)

1. Set environment variables locally:
```bash
export DATABASE_URL="your-database-url"
```

2. Run migration:
```bash
cd backend
alembic upgrade head
```

---

## Step 5: Ingest Content to Qdrant

**Important**: First time deployment ke baad content ingest karna hoga.

### Option A: Local Ingestion (Recommended - Faster)

1. Make sure you have all environment variables set:
```bash
export QDRANT_URL="https://your-cluster.qdrant.io"
export QDRANT_API_KEY="your-api-key"
```

2. Run ingestion:
```bash
cd backend
python scripts/ingest_content.py \
  --source ../book_frontend/docs \
  --create-collection \
  --force \
  --base-url https://hamnakh.github.io/Physical-AI-Humanoid-Robotics-Book
```

3. Wait for completion (~5-10 min for 872 chunks)

### Option B: Using Render Shell

```bash
cd backend
python scripts/ingest_content.py \
  --source ../book_frontend/docs \
  --create-collection \
  --force \
  --base-url https://hamnakh.github.io/Physical-AI-Humanoid-Robotics-Book
```

**Note**: Render shell me model download slow ho sakta hai.

---

## Step 6: Test Your Deployment

### 6.1 Get Your API URL

Render Dashboard me service ka URL dikh raha hoga:
- Format: `https://rag-chatbot-api.onrender.com`
- Ya custom name: `https://your-service-name.onrender.com`

### 6.2 Health Check

Browser me open karo ya curl se test karo:
```bash
curl https://your-service-name.onrender.com/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-07T..."
}
```

### 6.3 API Documentation

- **Swagger UI**: `https://your-service-name.onrender.com/docs`
- **ReDoc**: `https://your-service-name.onrender.com/redoc`

### 6.4 Test Chat Endpoint

```bash
curl -X POST https://your-service-name.onrender.com/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "What is ROS 2?",
    "mode": "full_book"
  }'
```

---

## Step 7: Update Frontend API URL

Frontend me production API URL update karo:

**File**: `book_frontend/src/plugins/rag-chatbot/api/chatClient.ts`

```typescript
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-service-name.onrender.com'  // Change this!
  : 'http://localhost:8000';
```

Then rebuild and redeploy frontend:
```bash
cd book_frontend
npm run build
git add .
git commit -m "Update API URL for production"
git push origin main
```

---

## Troubleshooting

### ❌ Build Fails

**Problem**: Dependencies install nahi ho rahi

**Solution**:
1. Check **Logs** tab in Render
2. Verify `requirements.txt` has all dependencies
3. Check Python version compatibility
4. Free tier memory limit: Reduce batch sizes if needed

### ❌ Service Won't Start

**Problem**: Application startup fails

**Solution**:
1. Check **Logs** tab for error messages
2. Verify all environment variables are set
3. Check `DATABASE_URL` format (should be valid PostgreSQL URL)
4. Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct

### ❌ Database Connection Error

**Problem**: Can't connect to database

**Solution**:
1. Verify `DATABASE_URL` is correct
2. For Render PostgreSQL: Use **Internal Database URL**
3. For Neon: Use pooled connection string (contains `-pooler`)
4. Check database is in same region as web service

### ❌ Qdrant Connection Error

**Problem**: Can't connect to Qdrant

**Solution**:
1. Verify `QDRANT_URL` format: `https://xxx.qdrant.io`
2. Check `QDRANT_API_KEY` is valid
3. Test connection from browser/curl
4. Verify cluster is active in Qdrant dashboard

### ❌ CORS Errors in Frontend

**Problem**: Frontend can't call API

**Solution**:
1. Check `CORS_ORIGINS` includes your frontend URL
2. Current setting: `http://localhost:3000,https://hamnakh.github.io`
3. If frontend URL different, update in Render dashboard
4. Redeploy after changing environment variables

### ❌ Slow First Request (Free Tier)

**Problem**: First request takes 30-60 seconds

**Solution**:
- This is normal! Free tier services sleep after 15 min inactivity
- First request wakes up the service (~30 sec)
- Model loads on first request (~30-60 sec)
- Subsequent requests are fast
- Upgrade to paid plan ($7/mo) for always-on

### ❌ No Search Results

**Problem**: Chat returns no results

**Solution**:
1. Verify content ingestion completed successfully
2. Check Qdrant collection exists: `book_content`
3. Verify collection has vectors: Check in Qdrant dashboard
4. Re-run ingestion if needed

---

## Quick Reference

### Render Service URL Format
```
https://[service-name].onrender.com
```

### Important URLs
- **API Root**: `https://your-service.onrender.com/`
- **Health Check**: `https://your-service.onrender.com/api/health`
- **API Docs**: `https://your-service.onrender.com/docs`
- **Session Endpoint**: `https://your-service.onrender.com/api/session`
- **Chat Endpoint**: `https://your-service.onrender.com/api/chat`

### Environment Variables Summary

**Required (Must set manually in Render):**
- `OPENAI_API_KEY`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `DATABASE_URL`

**Optional:**
- `SENTRY_DSN` (for error tracking)

**Already in render.yaml:**
- All other variables (CORS, rates, models, etc.)

---

## Free Tier Limitations

### Render Free Tier
- ✅ **Web Service**: 750 hours/month (enough for always-on)
- ⚠️ **Auto-sleep**: Spins down after 15 min inactivity
- ⚠️ **First request slow**: 30-60 sec wake-up time
- ✅ **PostgreSQL**: 90 days free, then $7/month
- ✅ **Bandwidth**: 100 GB/month

### Qdrant Free Tier
- ✅ 1 cluster free
- ✅ 1 GB storage
- ✅ Sufficient for this project

### Upgrade Options
- **Render Paid**: $7/month for always-on web service
- **PostgreSQL Paid**: $7/month after free trial
- Total: ~$14/month for production-ready setup

---

## Success Checklist

After deployment, verify:

- [ ] Service deployed successfully
- [ ] Health check returns 200 OK
- [ ] Database migrations ran
- [ ] Content ingested to Qdrant
- [ ] API docs accessible at `/docs`
- [ ] Frontend can connect to API
- [ ] Chat endpoint returns responses
- [ ] CORS configured correctly

---

## Next Steps

1. ✅ Backend deployed on Render
2. ✅ Frontend deployed on GitHub Pages
3. ✅ Both connected and working
4. 🔄 Monitor logs for errors
5. 🔄 Set up error tracking (Sentry - optional)
6. 🔄 Add custom domain (paid plans)

---

## Support Links

- **Render Docs**: https://render.com/docs
- **Render Status**: https://status.render.com
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **FastAPI Docs**: https://fastapi.tiangolo.com/

---

**Deployment Complete!** 🚀

Your API should be live at: `https://your-service-name.onrender.com`
