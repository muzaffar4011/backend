from qdrant_client import QdrantClient

client = QdrantClient(
    url="https://6e8e9678-1e3a-46ae-87df-65282d2a1317.us-east4-0.gcp.cloud.qdrant.io",
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.EYw3AFI_LzrZVYgAgcU-1Spd9JRd5cWoEo9hQvSVWMY"
)

collections = client.get_collections()
print(collections)

